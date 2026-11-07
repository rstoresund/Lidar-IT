"""Offline tests of the dependency bootstrapper; never installs packages."""
import sys
from pathlib import Path
from unittest.mock import patch
import dependency_bootstrap as d

# Packages are inspected without importing dependencies. Test missing detection.
with patch('dependency_bootstrap.importlib.util.find_spec', side_effect=lambda x: None if x in ('customtkinter','lazrs') else object()):
    missing = d.missing_dependencies()
    assert [m for m, _ in missing] == ['lazrs','customtkinter'], missing
    assert d.missing_requirements(missing) == ['laspy[lazrs]>=2.5,<3','customtkinter>=5.2,<6']

# Same executable as running process; don't accidentally install into a different Python.
assert Path(d.pip_command(['numpy>=1.26,<3'])[0]).resolve() == d.pip_python().resolve()
assert 'pip' in d.pip_command(['numpy>=1.26,<3'])

# When everything is installed, a check must not invoke pip or the installer GUI.
with patch('dependency_bootstrap.missing_dependencies',return_value=[]), \
     patch('dependency_bootstrap._gui_install',side_effect=AssertionError('unexpected installer')):
    assert d.ensure_dependencies()

# Verify the installation is attempted if there is a missing module; this is
# fully mocked so no package manager or download is touched during this test.
with patch('dependency_bootstrap.missing_dependencies',side_effect=[[('customtkinter','customtkinter>=5.2,<6')], []]), \
     patch('dependency_bootstrap._gui_install',return_value=True) as installer:
    assert d.ensure_dependencies()
    installer.assert_called_once_with([('customtkinter','customtkinter>=5.2,<6')],
                                      ['customtkinter>=5.2,<6'])

print('PASS: dependency detection, exact interpreter, install planning, no-op when satisfied')
