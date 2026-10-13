import gettext
from openpilot.system.ui.lib.multilang import (
  multilang as base_multilang,
  TRANSLATIONS_DIR,
  tr_noop,
  load_translations,
)


class _PoTranslations(gettext.NullTranslations):
  """dp - fallback when dragonpilot_<lang>.mo was never compiled (*.mo is gitignored and nothing in
  SCons builds it), so a fresh checkout still shows translated dp settings. Parses the .po directly
  with the same parser the base UI uses for app_<lang>.po."""
  def __init__(self, po_path):
    super().__init__()
    self._translations, self._plurals = load_translations(po_path)

  def gettext(self, message):
    return self._translations.get(message) or message

  def ngettext(self, msgid1, msgid2, n):
    forms = self._plurals.get(msgid1)
    if forms:
      idx = base_multilang._plural_selector(n) if hasattr(base_multilang, "_plural_selector") else 0
      if 0 <= idx < len(forms) and forms[idx]:
        return forms[idx]
    return msgid1 if n == 1 else msgid2


class DpMultilang:
  """Wrapper that syncs with base multilang and adds dragonpilot translations."""

  def __init__(self):
    self._dragon_translation: gettext.NullTranslations | gettext.GNUTranslations = gettext.NullTranslations()
    self._loaded_language: str = ""

  @property
  def languages(self):
    """Delegate to base multilang."""
    return base_multilang.languages

  @property
  def language(self):
    """Delegate to base multilang."""
    return base_multilang.language

  def _ensure_loaded(self):
    """Reload dragon translations if base language changed."""
    current_lang = base_multilang.language
    if current_lang != self._loaded_language:
      self._loaded_language = current_lang
      try:
        with TRANSLATIONS_DIR.joinpath(f'dragonpilot_{current_lang}.mo').open('rb') as fh:
          self._dragon_translation = gettext.GNUTranslations(fh)
      except FileNotFoundError:
        po_path = TRANSLATIONS_DIR.joinpath(f'dragonpilot_{current_lang}.po')
        try:
          self._dragon_translation = _PoTranslations(po_path) if po_path.is_file() else gettext.NullTranslations()
        except Exception:
          self._dragon_translation = gettext.NullTranslations()

  def tr(self, text: str) -> str:
    self._ensure_loaded()
    result = self._dragon_translation.gettext(text)
    return result if result != text else base_multilang.tr(text)

  def trn(self, singular: str, plural: str, n: int) -> str:
    self._ensure_loaded()
    result = self._dragon_translation.ngettext(singular, plural, n)
    return result if result not in (singular, plural) else base_multilang.trn(singular, plural, n)


multilang = DpMultilang()

tr, trn = multilang.tr, multilang.trn

__all__ = ['multilang', 'tr', 'trn', 'tr_noop', 'TRANSLATIONS_DIR']
