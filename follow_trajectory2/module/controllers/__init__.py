#https://stackoverflow.com/questions/1057431/how-to-load-all-modules-in-a-folder
from os.path import dirname, basename, isfile, join, isdir
import glob

__all__ = [ basename(f)[:-3] for f in glob.glob(join(dirname(__file__), "*")) if isfile(f) and not f.endswith('__init__.py') and not f.endswith('pyc') and not f.endswith("controllerabc.py") and not f.endswith("utils.py")]

