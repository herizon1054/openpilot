"""Jetlink ONNX cache operations; never touch the fork's model files."""
import fcntl
import os
import re
import stat
from contextlib import contextmanager
from pathlib import Path

_MODEL = re.compile(r"[0-9a-f]{16}\.onnx(?:\.part)?")


@contextmanager
def cache_lock(directory):
  directory = Path(directory)
  if directory.is_symlink():
    raise RuntimeError("The model cache must not be a symbolic link.")
  directory.mkdir(parents=True, exist_ok=True)
  fd = os.open(directory / '.cache.lock', os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
  try:
    try:
      fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      raise RuntimeError("Jetlink is busy. Wait for it to stop and try again.") from None
    yield
  finally:
    os.close(fd)


def files(directory):
  directory = Path(directory)
  if directory.is_symlink():
    raise RuntimeError("The model cache must not be a symbolic link.")
  if not directory.exists():
    return []
  result = []
  for path in directory.iterdir():
    if _MODEL.fullmatch(path.name):
      try:
        info = path.lstat()
      except FileNotFoundError:
        continue
      if stat.S_ISREG(info.st_mode):
        result.append((path.name, info.st_size))
  return sorted(result)


def clear(directory, allowed):
  """Offroad/link-off checked under locks; no glob-based recursive removal."""
  from jetlink.comma import gadget
  if not allowed():
    raise RuntimeError("Park and turn Jetlink off before deleting models.")
  gadget.OWNER_LOCK.parent.mkdir(parents=True, exist_ok=True)
  fd = os.open(gadget.OWNER_LOCK, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o644)
  try:
    try:
      fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      raise RuntimeError("Jetlink is busy. Wait for it to stop and try again.") from None
    with cache_lock(directory):
      if not allowed():
        raise RuntimeError("Park and turn Jetlink off before deleting models.")
      removed, size = 0, 0
      for name, nbytes in files(directory):
        if not allowed():
          raise RuntimeError("Park and turn Jetlink off before deleting models.")
        try:
          (Path(directory) / name).unlink()
        except FileNotFoundError:
          continue
        removed += 1
        size += nbytes
      return removed, size
  finally:
    os.close(fd)
