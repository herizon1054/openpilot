"""Offline catalog/compiler and language-switch checks.

Run: python3 tools/jetlink/test_translations.py
Requires GNU msgfmt; no camera, GPU, cereal or graphical display is needed.
"""
import ast
import gettext
import importlib.util
import json
from pathlib import Path
import re
import shutil
import string
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
CATALOGS = ROOT / 'selfdrive/ui/translations'
spec = importlib.util.spec_from_file_location('jetlink_potools', CATALOGS / 'potools.py')
po = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = po
spec.loader.exec_module(po)
FILES = ['dragonpilot/selfdrive/ui/jetlink_ui.py', 'dragonpilot/selfdrive/ui/layouts/settings/jetlink.py']
KEYS = {e.msgid for e in po.extract_strings(FILES, str(ROOT))}


def functions_from(path, names, env):
  tree = ast.parse((ROOT / path).read_text())
  nodes = [n for n in tree.body if getattr(n, 'name', None) in names]
  assert len(nodes) == len(names)
  exec(compile(ast.Module(body=nodes, type_ignores=[]), path, 'exec'), env)
  return env


def runtime():
  # Execute the real loader and wrapper with only Params/logging dependencies removed.
  env = functions_from('system/ui/lib/multilang.py', {'_parse_quoted', 'load_translations'}, {'re': re})
  base = SimpleNamespace(language='en', languages={}, tr=lambda text: text,
                         trn=lambda one, many, n: one if n == 1 else many)
  env.update(base_multilang=base, TRANSLATIONS_DIR=CATALOGS,
             PLURAL_SELECTORS={'zh-CHT': lambda n: 0, 'zh-CHS': lambda n: 0})
  functions_from('dragonpilot/system/ui/lib/multilang.py', {'DpMultilang'}, env)
  return base, env['DpMultilang'](), env['load_translations']


class TranslationTests(unittest.TestCase):
  def test_all_catalogs_compile(self):
    self.assertIsNotNone(shutil.which('msgfmt'), 'GNU msgfmt is required')
    catalogs = sorted(CATALOGS.glob('*.po'))
    self.assertGreaterEqual(len(catalogs), 25)
    with tempfile.TemporaryDirectory() as d:
      for catalog in catalogs:
        with self.subTest(catalog=catalog.name):
          result = subprocess.run(['msgfmt', '--check', '--check-format', str(catalog), '-o', d+'/out.mo'],
                                  capture_output=True, text=True)
          self.assertEqual(result.returncode, 0, result.stderr)
          with open(d+'/out.mo', 'rb') as stream:
            gettext.GNUTranslations(stream)

  def test_chinese_coverage_and_placeholders(self):
    self.assertGreaterEqual(len(KEYS), 69)
    formatter = string.Formatter()
    fields = lambda s: sorted((key, spec, conv) for _, key, spec, conv in formatter.parse(s) if key is not None)
    for lang in ('zh-CHT', 'zh-CHS'):
      _, entries = po.parse_po(CATALOGS / f'dragonpilot_{lang}.po')
      translations = {e.msgid: e.msgstr for e in entries}
      for key in KEYS:
        with self.subTest(language=lang, key=key):
          self.assertTrue(translations.get(key), 'missing translation')
          self.assertEqual(fields(key), fields(translations[key]))

  def test_live_switch_and_english_fallback_without_mo(self):
    base, translator, _ = runtime()
    for language, expected in [('zh-CHT', '未連線'), ('zh-CHS', '未连接'), ('en', 'Disconnected'),
                               ('zh-CHT', '未連線')]:
      base.language = language
      self.assertEqual(translator.tr('Disconnected'), expected)
    self.assertEqual(translator.tr('unknown diagnostic from server'), 'unknown diagnostic from server')

  def test_runtime_matches_compiled_chinese_catalogs(self):
    _, _, loader = runtime()
    with tempfile.TemporaryDirectory() as d:
      for lang in ('zh-CHT', 'zh-CHS'):
        path = CATALOGS / f'dragonpilot_{lang}.po'
        translated, _ = loader(path)
        subprocess.run(['msgfmt', '--check-format', str(path), '-o', d+'/out.mo'], check=True)
        with open(d+'/out.mo', 'rb') as stream:
          compiled = gettext.GNUTranslations(stream)
        for key in KEYS:
          self.assertEqual(translated[key], compiled.gettext(key), key)

  def test_merge_language_and_plural_count(self):
    # Both domains must keep one Chinese form and three Ukrainian forms after updates.
    with tempfile.TemporaryDirectory() as d:
      d = Path(d)
      template = d/'test.pot'
      po.generate_pot([po.POEntry(msgid='{} model', msgid_plural='{} models', msgstr_plural={0:'', 1:''})], template)
      for domain in ('app', 'dragonpilot'):
        for lang, count in [('zh-CHT',1), ('zh-CHS',1), ('uk',3), ('en',2)]:
          path = d/f'{domain}_{lang}.po'
          po.init_po(template, path, lang)
          po.merge_po(path, template)
          header, entries = po.parse_po(path)
          self.assertIn(f'Language: {lang}\n', header.msgstr)
          self.assertEqual(set(entries[0].msgstr_plural), set(range(count)))
          result = subprocess.run(['msgfmt', '--check', str(path), '-o', str(d/'out.mo')], capture_output=True, text=True)
          self.assertEqual(result.returncode, 0, result.stderr)

  def test_actual_update_pipeline(self):
    languages = json.loads((CATALOGS/'languages.json').read_text())
    with tempfile.TemporaryDirectory() as d:
      output = Path(d)
      for source in CATALOGS.glob('dragonpilot_*.po'):
        shutil.copyfile(source, output/source.name)
      env = dict(Path=Path, BASEDIR=str(ROOT), TRANSLATIONS_DIR=output,
                 multilang=SimpleNamespace(languages=languages), extract_strings=po.extract_strings,
                 generate_pot=po.generate_pot, merge_po=po.merge_po, init_po=po.init_po)
      functions_from('dragonpilot/selfdrive/ui/update_translations.py', {'update_translations'}, env)
      env['update_translations']()
      for language in languages.values():
        path = output/f'dragonpilot_{language}.po'
        result = subprocess.run(['msgfmt', '--check', '--check-format', str(path), '-o', str(output/'out.mo')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        header, entries = po.parse_po(path)
        self.assertIn(f'Language: {language}\n', header.msgstr)
        if language in ('zh-CHT', 'zh-CHS'):
          translated = {entry.msgid: entry.msgstr for entry in entries}
          self.assertTrue(all(translated.get(key) for key in KEYS))

  def test_extractor_retains_event_strings(self):
    _, entries = po.parse_po(CATALOGS/'dragonpilot.pot')
    self.assertTrue(KEYS <= {e.msgid for e in entries})
    for key in ('Switching large model', 'Large model active', 'Large model disconnected'):
      self.assertIn(key, KEYS)


if __name__ == '__main__':
  unittest.main(verbosity=2)
