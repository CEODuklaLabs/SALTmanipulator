[WEBAPP] set_mode failed: No module named 'lib16inpind'
Exception in thread Thread-1 (_control_loop):
Traceback (most recent call last):
  File "/usr/lib/python3.11/threading.py", line 1038, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.11/threading.py", line 975, in run
    self._target(*self._args, **self._kwargs)
  File "/home/saltmanipulator/SALTmanipulator/webapp.py", line 143, in _control_loop
    inputs = hw.read_all_inputs()
             ^^^^^^^^^^^^^^^^^^^^
  File "/home/saltmanipulator/SALTmanipulator/hardware.py", line 104, in read_all_inputs
    raw = self._lib_in.get_all(self._in_stack)
          ^^^^^^^^^^^^^^^^^^^^
AttributeError: 'NoneType' object has no attribute 'get_all'
