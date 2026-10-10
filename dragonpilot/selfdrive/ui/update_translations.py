#!/usr/bin/env python3
"""Update dragonpilot PO catalogs using the main UI's Python tooling.

Both UI domains load PO directly; no external gettext compiler or generated MO
is required on the device. GNU msgfmt can still validate/compile these catalogs.
"""
from pathlib import Path
from openpilot.common.basedir import BASEDIR
from dragonpilot.system.ui.lib.multilang import TRANSLATIONS_DIR, multilang
from openpilot.selfdrive.ui.translations.potools import extract_strings, generate_pot, merge_po, init_po


def update_translations():
  root = Path(BASEDIR)
  files = sorted(str(p.relative_to(root)) for p in (root / 'dragonpilot').rglob('*.py'))
  template = TRANSLATIONS_DIR / 'dragonpilot.pot'
  generate_pot(extract_strings(files, str(root)), template)
  for language in multilang.languages.values():
    catalog = TRANSLATIONS_DIR / f'dragonpilot_{language}.po'
    if catalog.exists():
      merge_po(catalog, template)
    else:
      init_po(template, catalog, language)


if __name__ == '__main__':
  update_translations()
