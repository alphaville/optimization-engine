import opengen as og

mng = og.tcp.OptimizerTcpManager('.python_test_build/tcp_enabled_optimizer')
mng.start()
print(mng.ping())
print(mng.ping())
print(mng.call([1.0, 2.0], buffer_len=2048))
mng.kill()
