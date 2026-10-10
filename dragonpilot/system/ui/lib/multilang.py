from openpilot.system.ui.lib.multilang import (
  multilang as base_multilang,
  TRANSLATIONS_DIR,
  PLURAL_SELECTORS,
  load_translations,
  tr_noop,
)


class DpMultilang:
  """Use the same PO loader and language selection as the main UI.

  Load source catalogs directly so a missing or stale generated MO cannot hide
  translations. Reload when the UI language changes, including while onroad.
  """

  def __init__(self):
    self._translations: dict[str, str] = {}
    self._plurals: dict[str, list[str]] = {}
    self._loaded_language = None

  @property
  def languages(self):
    return base_multilang.languages

  @property
  def language(self):
    return base_multilang.language

  def _ensure_loaded(self):
    language = base_multilang.language
    if language != self._loaded_language:
      try:
        translations, plurals = load_translations(TRANSLATIONS_DIR.joinpath(f'dragonpilot_{language}.po'))
      except FileNotFoundError:
        translations, plurals = {}, {}
      self._translations, self._plurals = translations, plurals
      self._loaded_language = language

  def tr(self, text: str) -> str:
    self._ensure_loaded()
    return self._translations.get(text) or base_multilang.tr(text)

  def trn(self, singular: str, plural: str, n: int) -> str:
    self._ensure_loaded()
    forms = self._plurals.get(singular, [])
    index = PLURAL_SELECTORS.get(self._loaded_language, lambda count: 0 if count == 1 else 1)(n)
    if 0 <= index < len(forms) and forms[index]:
      return forms[index]
    return base_multilang.trn(singular, plural, n)


multilang = DpMultilang()
tr, trn = multilang.tr, multilang.trn
__all__ = ['multilang', 'tr', 'trn', 'tr_noop', 'TRANSLATIONS_DIR']
