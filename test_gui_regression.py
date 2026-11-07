"""Static regression test for Tkinter / CustomTkinter GUI method collisions.

Run: py -3.12 test_gui_regression.py
No third-party packages or display required.
"""
import ast
import tkinter as tk
from pathlib import Path

source = Path(__file__).with_name('pointcloud_filter.py').read_text(encoding='utf8')
tree = ast.parse(source)
app = next(
    cls
    for function in tree.body if isinstance(function, (ast.FunctionDef, ast.AsyncFunctionDef)) and function.name == 'gui'
    for cls in function.body if isinstance(cls, ast.ClassDef) and cls.name == 'App'
)
names = {method.name for method in app.body if isinstance(method, (ast.FunctionDef, ast.AsyncFunctionDef))}
# Protect Tkinter internal methods; __init__ is intentionally overridden.
collisions = sorted((names - {'__init__'}).intersection(dir(tk.Tk)))
assert not collisions, f'App overrides reserved Tkinter methods: {collisions}'
assert 'get_filter_options' in names, names
assert '_options' not in names
assert 'self.get_filter_options()' in source
assert 'self._options()' not in source
print('PASS: GUI contains no Tkinter method collisions and calls get_filter_options()')
