function bytes = pack_pwm_u16_bytes(m1, m2, m3, m4)
%PACK_PWM_U16_BYTES Convert four motor values into 8 raw bytes.
%   BYTES = PACK_PWM_U16_BYTES(M1,M2,M3,M4) returns a uint8 vector of
%   length 8 encoded as little-endian <4H> suitable for the GUI's UDP 8888
%   PWM receiver.
%
%   Each input is cast to UINT16 before typecasting to UINT8. On
%   big-endian hosts, values are byte-swapped to maintain little-endian
%   network order.
%
%   Example:
%       u = udpport("byte", "OutputDatagramSize", 8);
%       write(u, pack_pwm_u16_bytes(1000,2000,3000,4000), "uint8", "127.0.0.1", 8888);
%
%   See also TYPECAST, UINT16, SWAPBYTES.

vals = uint16([m1, m2, m3, m4]);
% Detect host endianness; swap if running on big-endian hardware.
if ~isequal(typecast(uint16(1), 'uint8'), uint8([1 0]))
    vals = swapbytes(vals);
end
bytes = typecast(vals, 'uint8');
end
