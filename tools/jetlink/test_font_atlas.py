"""Check actual font glyph selection and SCons dependencies without a GPU."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from test_translations import po

ROOT = Path(__file__).resolve().parents[2]
FONTS = ROOT / 'selfdrive/assets/fonts'
CATALOGS = ROOT / 'selfdrive/ui/translations'


def generator():
  path = FONTS/'process.py'
  tree = ast.parse(path.read_text())
  # Import raylib only on the device. These are the unmodified generator functions.
  nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.Assign))]
  env = dict(Path=Path, json=json, __file__=str(path), rl=SimpleNamespace())
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), env)
  return env


class FontAtlasTests(unittest.TestCase):
  def test_all_cjk_dp_translations_are_in_generated_charsets(self):
    env = generator()
    _, _, per_lang = env['_char_sets']()
    for language, codepoints in per_lang.items():
      _, entries = po.parse_po(CATALOGS/f'dragonpilot_{language}.po')
      texts = [e.msgstr for e in entries] + [s for e in entries for s in e.msgstr_plural.values()]
      missing = {c for text in texts for c in text if c.isprintable() and ord(c) not in codepoints}
      self.assertFalse(missing, (language, missing))
    for code, sample in [('zh-CHT','來源選擇外部尚報建置引擎停車'), ('zh-CHS','来源选择外部尚报构建引擎停车')]:
      self.assertTrue(set(map(ord, sample)) <= set(per_lang[code]))

  def test_no_mo_dependency(self):
    env = generator()
    import tempfile
    with tempfile.TemporaryDirectory() as d:
      env['TRANSLATIONS_DIR'] = Path(d)
      (Path(d)/'dragonpilot_zh-CHT.po').write_text('msgid "Source"\nmsgstr "來源"\n')
      (Path(d)/'dragonpilot_zh-CHT.mo').write_bytes(b'invalid stale MO')
      self.assertTrue(set('來源') <= env['_dragonpilot_chars']('zh-CHT'))

  def test_build_targets_match_generator_and_include_po_inputs(self):
    env = generator()
    generated = []
    env['_process_font'] = lambda path, cp, output_name=None: generated.extend(
      f'selfdrive/assets/fonts/{output_name or path.stem}{ext}' for ext in ('.fnt', '.png'))
    self.assertEqual(env['main'](), 0)
    commands = []
    def file(name):
      relative = name.removeprefix('#')
      return SimpleNamespace(path=relative, name=Path(relative).name, abspath=str(ROOT/relative))
    scope = dict(Import=lambda *args:None, File=file,
                 Glob=lambda pattern:[file(str(p.relative_to(ROOT))) for p in ROOT.glob(pattern.removeprefix('#'))],
                 GetOption=lambda option:False, env=SimpleNamespace(Command=lambda **kw:commands.append(kw)))
    exec(compile((ROOT/'selfdrive/ui/SConscript').read_text(), 'SConscript', 'exec'), scope)
    self.assertEqual(len(commands), 1)
    self.assertEqual(set(generated), {s.removeprefix('#') for s in commands[0]['target']})
    sources = {n.path for n in commands[0]['source']}
    self.assertIn('selfdrive/ui/translations/languages.json', sources)
    for path in CATALOGS.glob('*.po'):
      self.assertIn(str(path.relative_to(ROOT)), sources)


if __name__ == '__main__':
  unittest.main(verbosity=2)
