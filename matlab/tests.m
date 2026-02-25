% Build + parse roundtrip test
rx = aarl_comms('rx_state_new');

payload = uint8([1 2 3 4 5]);
frm = aarl_comms('build_frame', 7, 9, payload);

[frames, rx] = aarl_comms('parse_stream', rx, frm);

assert(numel(frames) == 1);
f = frames{1};
assert(f.crc_ok);
assert(f.msg_type == uint8(7));
assert(f.seq_num  == uint8(9));
assert(isequal(f.payload, payload));
disp('roundtrip ok');