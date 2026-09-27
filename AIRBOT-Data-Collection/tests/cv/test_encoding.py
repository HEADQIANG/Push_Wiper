import os
import sys


print(os.environ.get("LANG"))
assert "UTF-8" in os.environ.get("LANG"), "系统默认编码不是UTF-8"
print(sys.getdefaultencoding())
assert sys.getdefaultencoding() == "utf-8", "Python默认编码不是UTF-8"
