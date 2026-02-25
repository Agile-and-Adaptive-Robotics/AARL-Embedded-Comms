function varargout = aarl_comms(cmd, varargin)
%AARL_COMMS Minimal comms core module (dispatcher + local functions).
%
% Frame format:
%   [SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]
%
% CRC16 is CCITT-FALSE:
%   poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
% CRC is computed over:
%   [TYPE][SEQ][LEN][PAYLOAD...]
% and appended as:
%   [CRClo][CRChi]  (little-endian)

    switch lower(string(cmd))
        case "sync"
            varargout{1} = uint8(hex2dec('AA'));
            if nargout > 1
                varargout{2} = uint8(hex2dec('55'));
            end

        case "crc16"
            varargout{1} = crc16_ccitt_false(varargin{:});

        case "build_frame"
            varargout{1} = build_frame(varargin{:});

        case "rx_state_new"
            varargout{1} = rx_state_new(varargin{:});

        case "parse_stream"
            [varargout{1}, varargout{2}] = parse_stream(varargin{:});

        otherwise
            error('aarl_comms:BadCmd', 'Unknown cmd: %s', string(cmd));
    end
end

% ============================
% Local constants (functions)
% ============================

function b = sync0_u8()
    b = uint8(hex2dec('AA'));
end

function b = sync1_u8()
    b = uint8(hex2dec('55'));
end

% ============================
% Local functions (private API)
% ============================

function rx_state = rx_state_new(max_buf)
%RX_STATE_NEW Create a streaming RX parser state.
%
%   rx_state = aarl_comms('rx_state_new')
%   rx_state = aarl_comms('rx_state_new', max_buf)
%
% rx_state fields (minimal):
%   buf     : uint8 row vector, rolling buffer
%   max_buf : max buffer length (bytes)
%   n_drop  : diagnostic counter (bytes dropped to enforce max_buf)

    if nargin < 1 || isempty(max_buf)
        max_buf = 8192;
    end

    if ~(isscalar(max_buf) && isnumeric(max_buf) && isfinite(max_buf) && max_buf == floor(max_buf) && max_buf >= 16)
        error('aarl_comms:BadMaxBuf', 'max_buf must be an integer scalar >= 16');
    end

    rx_state = struct();
    rx_state.buf     = uint8([]);     % always row
    rx_state.max_buf = double(max_buf);
    rx_state.n_drop  = double(0);
end

function [frames, rx_state] = parse_stream(rx_state, new_bytes)
%PARSE_STREAM Consume new bytes and emit any complete frames.
%
%   [frames, rx_state] = aarl_comms('parse_stream', rx_state, new_bytes)
%
% Inputs:
%   rx_state  : struct from rx_state_new()
%   new_bytes : uint8 vector (or numeric -> uint8)
%
% Output:
%   frames    : cell array of structs with fields:
%                .msg_type  (uint8)
%                .seq_num   (uint8)
%                .payload   (uint8 row)
%                .crc_ok    (logical)
%                .raw       (uint8 row)  % full raw frame bytes
%              NOTE: This emits only crc_ok==true frames by default.
%                    If you want to keep bad CRC frames too, change the filter below.

    if nargin < 2
        new_bytes = uint8([]);
    elseif ~isa(new_bytes, 'uint8')
        new_bytes = uint8(new_bytes);
    end
    new_bytes = new_bytes(:)';  % row

    % Append and enforce max buffer size (drop oldest)
    if ~isfield(rx_state, 'buf') || ~isfield(rx_state, 'max_buf')
        error('aarl_comms:BadState', 'rx_state is missing required fields. Use rx_state_new().');
    end

    rx_state.buf = [rx_state.buf, new_bytes];

    if numel(rx_state.buf) > rx_state.max_buf
        n_over = numel(rx_state.buf) - rx_state.max_buf;
        rx_state.buf = rx_state.buf((n_over+1):end);
        rx_state.n_drop = rx_state.n_drop + n_over;
    end

    frames = {};

    % Repeatedly try to extract frames
    while true
        [frame_struct, rx_state.buf, did_extract] = try_extract_one(rx_state.buf);
        if ~did_extract
            break;
        end

        % Default policy: keep only good CRC frames
        if frame_struct.crc_ok
            frames{end+1} = frame_struct; %#ok<AGROW>
        end
    end
end

function [frame_struct, buf_out, did_extract] = try_extract_one(buf_in)
%TRY_EXTRACT_ONE Attempt to extract one complete frame from buf_in.
%
% Returns:
%   did_extract = true if we either:
%     - extracted a frame (good or bad CRC), OR
%     - dropped bytes due to resync work
%   did_extract = false if we cannot make progress (need more bytes)

    SY0 = sync0_u8();
    SY1 = sync1_u8();

    buf = buf_in;
    frame_struct = struct('msg_type', uint8(0), 'seq_num', uint8(0), ...
                          'payload', uint8([]), 'crc_ok', false, 'raw', uint8([]));
    did_extract = false;

    % Need at least sync + header (2 + 3)
    if numel(buf) < 5
        buf_out = buf;
        return;
    end

    % Find SYNC0 SYNC1
    sync_idx = find_sync(buf, SY0, SY1);
    if isempty(sync_idx)
        % No sync found: keep last byte if it could be SY0, else drop all.
        if buf(end) == SY0
            buf_out = buf(end);  % keep potential start
        else
            buf_out = uint8([]);
        end
        did_extract = true;
        return;
    end

    % Drop anything before sync
    if sync_idx > 1
        buf = buf(sync_idx:end);
        did_extract = true;
    end

    % Need full header after sync
    if numel(buf) < 5
        buf_out = buf;
        return;
    end

    % Header fields
    msg_type = buf(3);
    seq_num  = buf(4);
    len_u8   = buf(5);
    payload_len = double(len_u8);

    % Total frame length
    % 2 sync + 3 hdr + payload + 2 crc
    frame_len = 2 + 3 + payload_len + 2;

    if numel(buf) < frame_len
        buf_out = buf;
        return; % wait for more bytes
    end

    raw_frame = buf(1:frame_len);
    buf_rem   = buf((frame_len+1):end);

    % Compute CRC over [TYPE SEQ LEN PAYLOAD...]
    crc_input = raw_frame(3:(3+3+payload_len-1)); % indices 3..(5+payload_len)
    crc_calc  = crc16_ccitt_false(crc_input);

    crc_lo = raw_frame(end-1);
    crc_hi = raw_frame(end);
    crc_rx = bitor(uint16(crc_lo), bitshift(uint16(crc_hi), 8));

    crc_ok = (crc_calc == crc_rx);

    payload = uint8([]);
    if payload_len > 0
        payload = raw_frame(6:(6+payload_len-1));
    end

    frame_struct.msg_type = msg_type;
    frame_struct.seq_num  = seq_num;
    frame_struct.payload  = payload(:)'; % row
    frame_struct.crc_ok   = logical(crc_ok);
    frame_struct.raw      = raw_frame(:)';

    buf_out = buf_rem;
    did_extract = true;

    % If CRC bad, attempt resync by discarding first byte and retry later.
    % This is conservative: it prevents getting stuck on a false sync.
    if ~crc_ok
        % Put remainder back plus everything after first byte of this raw_frame,
        % so we slide window by 1 byte.
        buf_out = [raw_frame(2:end), buf_rem];
    end
end

function idx = find_sync(buf, SY0, SY1)
%FIND_SYNC Find first occurrence of [SY0 SY1] in buf.
    idx = [];
    if numel(buf) < 2
        return;
    end
    % search SY0 then verify next byte
    candidates = find(buf(1:end-1) == SY0);
    for k = 1:numel(candidates)
        i = candidates(k);
        if buf(i+1) == SY1
            idx = i;
            return;
        end
    end
end

function frame_bytes = build_frame(msg_type, seq_num, payload_bytes)
%BUILD_FRAME Build [SYNC0 SYNC1 TYPE SEQ LEN PAYLOAD CRClo CRChi]
%
% Frame format:
%   [SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]
%
% CRC16 is CCITT-FALSE:
%   poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
% CRC is computed over:
%   [TYPE][SEQ][LEN][PAYLOAD...]
% and appended as:
%   [CRClo][CRChi]  (little-endian)

    msg_u8 = require_u8_scalar(msg_type, 'msg_type');
    seq_u8 = require_u8_scalar(seq_num,  'seq_num');

    if nargin < 3 || isempty(payload_bytes)
        payload_u8 = uint8([]);
    else
        payload_u8 = uint8(payload_bytes);
    end

    payload_len = numel(payload_u8);
    if payload_len > 255
        error('aarl_comms:PayloadTooLong', 'payload too long for u8 LEN');
    end

    header_wo_sync = uint8([msg_u8, seq_u8, uint8(payload_len)]);
    crc_input      = [header_wo_sync, payload_u8(:)'];   % row vector

    crc_value_u16 = crc16_ccitt_false(crc_input);
    crc_lo_u8     = uint8(bitand(crc_value_u16, uint16(255)));
    crc_hi_u8     = uint8(bitand(bitshift(crc_value_u16, -8), uint16(255)));

    frame_bytes = uint8([sync0_u8(), sync1_u8(), crc_input, crc_lo_u8, crc_hi_u8]);
end

function crc_value = crc16_ccitt_false(data_bytes, init_value)
%CRC16_CCITT_FALSE CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000)
%
% Frame format:
%   [SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]
%
% CRC is computed over:
%   [TYPE][SEQ][LEN][PAYLOAD...]
% and appended as:
%   [CRClo][CRChi]  (little-endian)

    if nargin < 2 || isempty(init_value)
        init_value = hex2dec('FFFF');
    end

    if ~isa(data_bytes, 'uint8')
        data_bytes = uint8(data_bytes);
    end

    crc_value = uint16(bitand(uint32(init_value), uint32(65535)));

    for i = 1:numel(data_bytes)
        byte_val  = uint16(data_bytes(i));
        crc_value = bitxor(crc_value, bitshift(byte_val, 8));

        for k = 1:8
            if bitand(crc_value, uint16(hex2dec('8000'))) ~= 0
                crc_value = bitxor(bitshift(crc_value, 1), uint16(hex2dec('1021')));
            else
                crc_value = bitshift(crc_value, 1);
            end
        end
    end

    crc_value = bitand(crc_value, uint16(hex2dec('FFFF')));
end

function u8 = require_u8_scalar(val, name_str)
%REQUIRE_U8_SCALAR Validate scalar integer in [0,255] and return uint8.

    if ~(isscalar(val) && isnumeric(val) && isfinite(val))
        error('aarl_comms:BadType', '%s must be a finite numeric scalar', name_str);
    end

    if val ~= floor(val)
        error('aarl_comms:BadType', '%s must be an integer scalar', name_str);
    end

    if val < 0 || val > 255
        error('aarl_comms:Range', '%s must be in [0,255]', name_str);
    end

    u8 = uint8(val);
end