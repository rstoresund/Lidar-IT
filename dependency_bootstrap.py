"""Dependency bootstrapper for PointCloud Filter 3.0.

Uses ONLY Python's standard library; safe to import before NumPy or CustomTkinter.
Installs missing packages into the SAME Python installation that started the app,
including Python IDLE's interpreter. Never installs or requires FARO software.
"""
from __future__ import annotations

import importlib
import importlib.util
import os
import queue
import site
import subprocess
import sys
import threading
from pathlib import Path

# import name -> pip requirement (LAS/LAZ both use the laspy[lazrs] extra)
DEPENDENCIES = (
    ('numpy', 'numpy>=1.26,<3'),
    ('pye57', 'pye57>=0.4.19,<0.6'),
    ('laspy', 'laspy[lazrs]>=2.5,<3'),
    ('lazrs', 'laspy[lazrs]>=2.5,<3'),
    ('customtkinter', 'customtkinter>=5.2,<6'),
    ('PIL', 'Pillow>=10,<13'),
    ('duckdb', 'duckdb>=1.2,<2'),
)


def missing_dependencies() -> list[tuple[str, str]]:
    """Return modules that Python cannot locate without importing packages."""
    importlib.invalidate_caches()
    return [(module, requirement) for module, requirement in DEPENDENCIES
            if importlib.util.find_spec(module) is None]


def missing_requirements(missing: list[tuple[str, str]]) -> list[str]:
    """Deduplicate pip targets; both LAS and LAZ depend on the same extra."""
    return list(dict.fromkeys(requirement for _, requirement in missing))


def pip_python() -> Path:
    """Prefer python.exe over pythonw.exe so pip can report its output on Windows."""
    executable = Path(sys.executable)
    if os.name == 'nt' and executable.name.lower() == 'pythonw.exe':
        console_executable = executable.with_name('python.exe')
        if console_executable.is_file():
            return console_executable
    return executable


def pip_command(requirements: list[str]) -> list[str]:
    return [str(pip_python()), '-m', 'pip', 'install', '--disable-pip-version-check',
            '--no-input', '--prefer-binary', *requirements]


def _refresh_python_paths() -> None:
    """Make user-installed packages visible to current IDLE session if necessary."""
    importlib.invalidate_caches()
    if site.ENABLE_USER_SITE:
        try:
            user_dir = site.getusersitepackages()
            if Path(user_dir).is_dir() and user_dir not in sys.path:
                site.addsitedir(user_dir)
        except (OSError, AttributeError):
            pass
    importlib.invalidate_caches()


def _start_process(cmd: list[str]) -> subprocess.Popen:
    flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0) if os.name == 'nt' else 0
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            encoding='utf-8', errors='replace', bufsize=1,
                            creationflags=flags)


def _console_install(requirements: list[str]) -> bool:
    cmd = pip_command(requirements)
    print('PointCloud Filter: installing missing dependencies:', ', '.join(requirements), flush=True)
    try:
        completed = subprocess.run(cmd, check=False)
    except OSError as exc:
        print(f'Unable to run pip with current Python: {exc}', file=sys.stderr)
        return False
    _refresh_python_paths()
    if completed.returncode != 0:
        print('Installation failed. Re-run from a command prompt:', file=sys.stderr)
        print('  ' + subprocess.list2cmdline(cmd), file=sys.stderr)
        return False
    return True


def _gui_install(missing: list[tuple[str,str]], requirements: list[str]) -> bool:
    """Show a small, responsive standard-Tk window while pip installs packages."""
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title('PointCloud Filter | First-run setup')
    root.geometry('660x360')
    root.minsize(550, 300)
    root.configure(bg='#18212c')
    root.resizable(True, True)

    title = tk.Label(root, text='Installing missing Python packages', bg='#18212c',
                     fg='white', font=('Segoe UI', 15, 'bold'), anchor='w')
    title.pack(fill='x', padx=18, pady=(15, 4))
    names = ', '.join(module for module, _ in missing)
    tk.Label(root, text='Needed: ' + names + '\n'
             'Using your current Python interpreter: ' + str(pip_python()),
             bg='#18212c', fg='#d1d9e2', justify='left', anchor='w',
             wraplength=610, font=('Segoe UI', 10)).pack(fill='x', padx=18, pady=(0, 12))
    bar = ttk.Progressbar(root, mode='indeterminate', length=610)
    bar.pack(fill='x', padx=18, pady=(0, 10))
    bar.start(12)
    out = tk.Text(root, bg='#0f1722', fg='#d9e3ed', insertbackground='white',
                  relief='flat', font=('Consolas', 9), height=9, wrap='word')
    out.pack(fill='both', expand=True, padx=18, pady=(0, 8))
    out.insert('end', 'Checking/installing dependencies. An internet connection may be required.\n')
    out.configure(state='disabled')
    footer = tk.Frame(root, bg='#18212c')
    footer.pack(fill='x', padx=18, pady=(0, 14))
    status = tk.Label(footer, text='Installingâ€¦', bg='#18212c', fg='#d1d9e2',
                      font=('Segoe UI', 10))
    status.pack(side='left')
    button = tk.Button(footer, text='Cancel', width=12)
    button.pack(side='right')

    events: queue.Queue[tuple[str, object]] = queue.Queue()
    proc: list[subprocess.Popen | None] = [None]
    cancelled = threading.Event()
    result = {'success': False, 'finished': False}

    def append_log(s: str) -> None:
        out.configure(state='normal')
        out.insert('end', s)
        out.see('end')
        # Bound the visible log to avoid consuming unnecessary RAM.
        if int(out.index('end-1c').split('.')[0]) > 250:
            out.delete('1.0', '60.0')
        out.configure(state='disabled')

    def abort() -> None:
        if result['finished']:
            root.destroy()
            return
        cancelled.set()
        status.configure(text='Cancelling installationâ€¦')
        button.configure(state='disabled')
        if proc[0] is not None and proc[0].poll() is None:
            proc[0].terminate()

    button.configure(command=abort)
    root.protocol('WM_DELETE_WINDOW', abort)

    def worker() -> None:
        try:
            cmd = pip_command(requirements)
            events.put(('line', '> ' + subprocess.list2cmdline(cmd) + '\n'))
            if cancelled.is_set():
                events.put(('done', False))
                return
            process = _start_process(cmd)
            proc[0] = process
            if cancelled.is_set():
                process.terminate()
            assert process.stdout is not None
            for line in process.stdout:
                events.put(('line', line))
            code = process.wait()
            events.put(('done', code == 0 and not cancelled.is_set()))
        except Exception as exc:
            events.put(('line', f'Installation error: {exc}\n'))
            events.put(('done', False))

    def poll() -> None:
        try:
            while True:
                kind, value = events.get_nowait()
                if kind == 'line':
                    append_log(str(value))
                elif kind == 'done':
                    result['finished'] = True
                    bar.stop()
                    _refresh_python_paths()
                    remaining = missing_dependencies()
                    result['success'] = bool(value) and not remaining
                    if result['success']:
                        status.configure(text='Setup complete â€” launching PointCloud Filter', fg='#90dfb5')
                        button.configure(text='Close', state='normal')
                        root.after(850, root.destroy)
                    else:
                        if remaining and value:
                            append_log('Installed packages are not visible to this interpreter: '
                                       + ', '.join(m for m, _ in remaining) + '\n')
                        append_log('\nSETUP FAILED. Run the pip command shown above in PowerShell '
                                   'or check your network/permissions.\n')
                        status.configure(text='Installation failed', fg='#ffadad')
                        button.configure(text='Close', state='normal')
        except queue.Empty:
            pass
        if not result['finished']:
            root.after(100, poll)

    threading.Thread(target=worker, daemon=True).start()
    root.after(100, poll)
    root.mainloop()
    return result['success']


def check_dependencies() -> bool:
    """Non-installing diagnostic for `python pointcloud_filter.py --check-deps`."""
    missing = missing_dependencies()
    if not missing:
        print('PASS: All PointCloud Filter Python dependencies are available.')
        return True
    print('Missing:', ', '.join(module for module, _ in missing))
    print('Install with: ' + subprocess.list2cmdline(pip_command(missing_requirements(missing))))
    return False


def ensure_dependencies() -> bool:
    """Automatically install missing modules, showing Tk progress if supported."""
    missing = missing_dependencies()
    if not missing:
        return True
    requirements = missing_requirements(missing)
    print('PointCloud Filter: missing', ', '.join(name for name, _ in missing), flush=True)
    # Built-in Tkinter is included in standard CPython Windows installers.
    try:
        success = _gui_install(missing, requirements)
    except Exception as exc:
        print(f'Installer window unavailable ({exc}); using console instead.', flush=True)
        success = _console_install(requirements)
    if not success:
        return False
    remaining = missing_dependencies()
    if remaining:
        print('Packages still unavailable in the running Python:',
              ', '.join(m for m, _ in remaining), file=sys.stderr)
        return False
    return True
