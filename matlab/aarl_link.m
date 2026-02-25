function varargout = aarl_link(cmd, varargin)
%AARL_LINK Serial link utilities: open/close, tx with seq, rx poll.

    switch lower(string(cmd))
        case "open"
            varargout{1} = open_link(varargin{:});

        case "close"
            varargout{1} = close_link(varargin{:});

        case "tx"
            % [seq, link] = aarl_link('tx', link, msg_type, payload_u8, seq_opt)
            [varargout{1}, varargout{2}] = tx(varargin{:});

        case "rx_poll"
            % [frames, link] = aarl_link('rx_poll', link, max_read)
            [varargout{1}, varargout{2}] = rx_poll(varargin{:});

        case "wait_for"
            % fr = aarl_link('wait_for', link, expect_type, expect_seq, timeout_s)
            varargout{1} = wait_for(varargin{:});

        otherwise
            error('aarl_link:BadCmd', 'Unknown cmd: %s', string(cmd));
    end
end

% ============================
% Local functions
% ============================

function link = open_link(port, baud)
    if nargin < 2 || isempty(baud)
        baud = 2000000; % aarl_comms MATLAB/C++/Python default
    end

    sp = serialport(port, baud);
    sp.Timeout = 0.01;
    flush(sp);

    link = struct();
    link.sp       = sp;
    link.rx_state = aarl_comms('rx_state_new');
    link.seq_tx   = uint8(0);
end

function link = close_link(link)
    if isfield(link, 'sp') && ~isempty(link.sp)
        try, flush(link.sp); catch, end
        try, delete(link.sp); catch, end
    end
    link.sp = [];
end

function [seq, link] = tx(link, msg_type, payload_u8, seq_opt)
    if nargin < 4 || isempty(seq_opt)
        seq = link.seq_tx;
        link.seq_tx = uint8(mod(double(link.seq_tx) + 1, 256));
    else
        seq = uint8(seq_opt);
    end

    if nargin < 3 || isempty(payload_u8)
        payload_u8 = uint8([]);
    elseif ~isa(payload_u8, 'uint8')
        payload_u8 = uint8(payload_u8);
    end
    payload_u8 = payload_u8(:)';

    frame = aarl_comms('build_frame', msg_type, seq, payload_u8);
    write(link.sp, frame, "uint8");
end

function [frames, link] = rx_poll(link, max_read)
    if nargin < 2 || isempty(max_read)
        max_read = 4096;
    end

    n_wait = link.sp.NumBytesAvailable;
    if n_wait <= 0
        frames = {};
        return;
    end

    read_n = min(n_wait, max_read);
    new_bytes = read(link.sp, read_n, "uint8");
    [frames, link.rx_state] = aarl_comms('parse_stream', link.rx_state, new_bytes);
end

function fr = wait_for(link, expect_type, expect_seq, timeout_s)
    if nargin < 4 || isempty(timeout_s)
        timeout_s = 0.5;
    end

    t0 = tic;
    fr = [];
    while toc(t0) < timeout_s
        [frames, link] = rx_poll(link); %#ok<ASGLU>
        for i = 1:numel(frames)
            f = frames{i};
            if f.msg_type == uint8(expect_type) && f.seq_num == uint8(expect_seq)
                fr = f;
                return;
            end
        end
    end
end