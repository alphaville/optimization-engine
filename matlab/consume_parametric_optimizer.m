function [out, json_response] = consume_parametric_optimizer(udp_connection, p)

p_formatted_str = sprintf('%f, ', p(1:end-1));
req_str = sprintf('{"parameter":[%s %f]}', p_formatted_str,p(end));
fwrite(udp_connection, req_str);
json_response = fread(udp_connection, 100000, 'char');
json_response = char(json_response');
out = jsondecode(json_response);
