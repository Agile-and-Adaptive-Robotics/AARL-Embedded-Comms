function varargout = aarl_protocol(cmd, varargin)
%AARL_PROTOCOL Message IDs and payload pack/unpack helpers for Muscle Mutt sense link.
%
% Types (from muscle_mutt_msg_types.h):
%   CMD_PING           = 0x01
%   CMD_SET_STREAM_US  = 0x11   payload: uint32 little-endian
%   CMD_STREAM_ENABLE  = 0x12   payload: uint8 (0/1)
%   CMD_SET_SPIKE_MASK = 0x21   (not implemented here)
%
%   PONG               = 0x02
%   SENSE_FRAME        = 0x20   payload: 36 bytes (layout TBD)
%   ACKNOWLEDGED       = 0x7F   payload: uint8 status

    switch lower(string(cmd))
        case "msg_ids"
            varargout{1} = msg_ids();

        case "pack"
            % payload = aarl_protocol('pack', msg_type, data_struct)
            varargout{1} = pack_payload(varargin{:});

        case "unpack"
            % data_struct = aarl_protocol('unpack', msg_type, payload_u8)
            varargout{1} = unpack_payload(varargin{:});

        case "build_cmd"
            % frame = aarl_protocol('build_cmd', msg_type, seq_num, data_struct)
            varargout{1} = build_cmd(varargin{:});

        case "cmd_ping"
            % frame = aarl_protocol('cmd_ping', seq_num)
            seq = varargin{1};
            varargout{1} = aarl_comms('build_frame', msg_ids().CMD_PING, seq, uint8([]));

        case "cmd_set_stream_us"
            % frame = aarl_protocol('cmd_set_stream_us', seq_num, period_us_u32)
            seq = varargin{1};
            period_us = varargin{2};
            payload = pack_u32_le(period_us);
            varargout{1} = aarl_comms('build_frame', msg_ids().CMD_SET_STREAM_US, seq, payload);

        case "cmd_stream_enable"
            % frame = aarl_protocol('cmd_stream_enable', seq_num, enable_bool)
            seq = varargin{1};
            enable = varargin{2};
            payload = uint8(enable ~= 0);
            varargout{1} = aarl_comms('build_frame', msg_ids().CMD_STREAM_ENABLE, seq, payload);

        case "sense_labels"
            varargout{1} = sense_labels();

        case "decode_sense_frame"
            % out = aarl_protocol('decode_sense_frame', payload_u8)
            varargout{1} = decode_sense_frame(varargin{:});
        
        otherwise
            error('aarl_protocol:BadCmd', 'Unknown cmd: %s', string(cmd));
    end
end

% ============================
% Local functions
% ============================

function ids = msg_ids()
    ids = struct();
    % host -> mcu
    ids.CMD_PING          = uint8(hex2dec('01'));
    ids.CMD_SET_STREAM_US = uint8(hex2dec('11'));
    ids.CMD_STREAM_ENABLE = uint8(hex2dec('12'));
    ids.CMD_SET_SPIKE_MASK= uint8(hex2dec('21')); % future use; not implemented here

    % mcu -> host
    ids.PONG              = uint8(hex2dec('02'));
    ids.SENSE_FRAME       = uint8(hex2dec('20'));
    ids.ACKNOWLEDGED      = uint8(hex2dec('7F'));
end

function frame = build_cmd(msg_type, seq_num, data_struct)
    payload = pack_payload(msg_type, data_struct);
    frame   = aarl_comms('build_frame', msg_type, seq_num, payload);
end

function payload = pack_payload(msg_type, data_struct)
    ids = msg_ids();
    mt = require_u8_scalar(msg_type, 'msg_type');

    switch mt
        case ids.CMD_PING
            payload = uint8([]);

        case ids.CMD_SET_STREAM_US
            if ~isfield(data_struct, 'period_us')
                error('aarl_protocol:BadPack', 'CMD_SET_STREAM_US expects data_struct.period_us');
            end
            payload = pack_u32_le(data_struct.period_us);

        case ids.CMD_STREAM_ENABLE
            if ~isfield(data_struct, 'enable')
                error('aarl_protocol:BadPack', 'CMD_STREAM_ENABLE expects data_struct.enable');
            end
            payload = uint8(data_struct.enable ~= 0);

        otherwise
            error('aarl_protocol:BadPack', 'No pack rule for msg_type=0x%02X', mt);
    end
end

function data_struct = unpack_payload(msg_type, payload_u8)
    ids = msg_ids();
    mt = require_u8_scalar(msg_type, 'msg_type');

    if ~isa(payload_u8, 'uint8')
        if ~isnumeric(payload_u8)
            error('aarl_protocol:BadPayload', 'Payload must be numeric or uint8');
        end
        payload_u8 = uint8(payload_u8);
    end
    payload_u8 = payload_u8(:)';   % force row vector

    switch mt
        case ids.PONG
            data_struct = struct(); % empty payload

        case ids.ACKNOWLEDGED
            status = uint8([]);
            if ~isempty(payload_u8)
                status = payload_u8(1);
            end
            data_struct = struct('status', status);

        case ids.SENSE_FRAME
            data_struct = decode_sense_frame(payload_u8);

        otherwise
            data_struct = struct('raw', payload_u8);
    end
end

function b = pack_u32_le(x)
%PACK_U32_LE Pack scalar into 4 bytes little-endian.
    if ~(isscalar(x) && isnumeric(x) && isfinite(x) && x == floor(x) && x >= 0 && x <= double(2^32-1))
        error('aarl_protocol:BadU32', 'Value must be integer scalar in [0, 2^32-1]');
    end
    u = uint32(x);
    b = typecast(u, 'uint8');  % little-endian on Windows (matches Python "<I")
    b = b(:)';                 % row
end

function u8 = require_u8_scalar(val, name_str)
    if ~(isscalar(val) && isnumeric(val) && isfinite(val))
        error('aarl_protocol:BadType', '%s must be a finite numeric scalar', name_str);
    end
    if val ~= floor(val)
        error('aarl_protocol:BadType', '%s must be an integer scalar', name_str);
    end
    if val < 0 || val > 255
        error('aarl_protocol:Range', '%s must be in [0,255]', name_str);
    end
    u8 = uint8(val);
end

function labels = sense_labels()
%SENSE_LABELS Names corresponding to the 36-byte SENSE_FRAME payload.
%
% First 12 = potentiometers (joint angles)
% Next 24  = pressure sensors (muscle pressures)

    labels = struct();

    labels.pots = string([ ...
        "L_hip_joint","L_knee_joint","L_ankle_joint", ...
        "R_hip_joint","R_knee_joint","R_ankle_joint", ...
        "L_scapula_joint","L_shoulder_joint","L_wrist_joint", ...
        "R_scapula_joint","R_shoulder_joint","R_wrist_joint" ...
    ]);

    labels.press = string([ ...
        "R_hip_ext","R_hip_flx","R_knee_ext","R_knee_flx","R_ank_ext","R_ank_flx", ...
        "L_hip_ext","L_hip_flx","L_knee_ext","L_knee_flx","L_ank_ext","L_ank_flx", ...
        "R_sca_ext","R_sca_flx","R_sho_ext","R_sho_flx","R_wri_ext","R_wri_flx", ...
        "L_sca_ext","L_sca_flx","L_sho_ext","L_sho_flx","L_wri_ext","L_wri_flx" ...
    ]);
end

function out = decode_sense_frame(payload_u8)
%DECODE_SENSE_FRAME Split 36-byte SENSE_FRAME payload into pot + press vectors.
%
% This stays intentionally low-level: raw uint8 values.

    if ~isa(payload_u8, 'uint8')
        if ~isnumeric(payload_u8)
            error('aarl_protocol:BadPayload', 'payload must be numeric/uint8');
        end
        payload_u8 = uint8(payload_u8);
    end
    payload_u8 = payload_u8(:)';

    if numel(payload_u8) ~= 36
        error('aarl_protocol:BadSenseLen', 'SENSE_FRAME payload must be 36 bytes (got %d)', numel(payload_u8));
    end

    out = struct();
    out.raw   = payload_u8;
    out.pots  = payload_u8(1:12);
    out.press = payload_u8(13:36);
end