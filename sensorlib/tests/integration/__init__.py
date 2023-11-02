# Add PWD to the path
import os
import sys

print("Running tests/integration/__init__.py")

current_dir = os.path.dirname(os.path.abspath(__file__))
lib_dir = os.path.join(current_dir, "../../lib")
sys.path.append(lib_dir)
