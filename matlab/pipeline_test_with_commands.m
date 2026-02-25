function pipeline_test_with_commands(port, baud)
%PIPELINE_TEST_WITH_COMMANDS MATLAB mirror of muscle_mutt_sense.pipeline_test_with_commands().
%
% Steps:
%   1) PING -> PONG
%   2) SET_STREAM_US(5000) -> ACK
%   3) STREAM_ENABLE(1) -> ACK
%   4) Count SENSE_FRAME payload_len==36 for 10 seconds

    if nargin < 1 || isempty(port), port = "COM3"; end
    if nargin < 2 || isempty(baud), baud = 2000000; end

    ids = aarl_protocol('msg_ids');

    link = aarl_link('open', port, baud);
    cleaner = onCleanup(@() aarl_link('close', link)); %#ok<NASGU>

    % 1) PING -> PONG
    [seq_ping, link] = aarl_link('tx', link, ids.CMD_PING, uint8([]));
    fr_pong = aarl_link('wait_for', link, ids.PONG, seq_ping, 0.5);
    fprintf('PING seq=%u -> %s\n', seq_ping, ternary(~isempty(fr_pong), "PONG OK", "PONG TIMEOUT"));

    % 2) Set stream period -> ACK
    target_us = 5000;
    payload_set = typecast(uint32(target_us), 'uint8'); payload_set = payload_set(:)'; % <I
    [seq_set, link] = aarl_link('tx', link, ids.CMD_SET_STREAM_US, payload_set);
    fr_ack1 = aarl_link('wait_for', link, ids.ACKNOWLEDGED, seq_set, 0.5);
    status1 = get_ack_status(fr_ack1);
    fprintf('SET_STREAM_US(%u) seq=%u -> ACK status=%s\n', target_us, seq_set, status_to_str(status1));

    % 3) Enable stream -> ACK
    [seq_en, link] = aarl_link('tx', link, ids.CMD_STREAM_ENABLE, uint8(1));
    fr_ack2 = aarl_link('wait_for', link, ids.ACKNOWLEDGED, seq_en, 0.5);
    status2 = get_ack_status(fr_ack2);
    fprintf('STREAM_ENABLE(1) seq=%u -> ACK status=%s\n', seq_en, status_to_str(status2));

    % 4) Count SENSE_FRAME frames for 10 seconds
    intact = 0;
    t0 = tic;
    while toc(t0) < 10.0
        [frames, link] = aarl_link('rx_poll', link, 4096);
        for i = 1:numel(frames)
            fr = frames{i};
            if fr.msg_type == ids.SENSE_FRAME && numel(fr.payload) == 36
                intact = intact + 1;
            end
        end
    end

    fprintf('\n\tMessages received during test: %d\n', intact);
    fprintf('\tPipeline speed: %.2f Hz\n\n', intact/10.0);
end

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end

function status = get_ack_status(fr)
    status = [];
    if ~isempty(fr) && isfield(fr, 'payload') && numel(fr.payload) >= 1
        status = fr.payload(1);
    end
end

function out = status_to_str(status)
    if isempty(status)
        out = "None";
    else
        out = string(double(status));
    end
end