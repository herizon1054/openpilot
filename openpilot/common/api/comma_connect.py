import os

from openpilot.common.api.base import BaseApi
from openpilot.sunnypilot.privacy import privacy_mode, PRIVACY_ALLOWED_ENDPOINTS

API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com')


class CommaConnectApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, API_HOST)
    self.user_agent = "openpilot-"

  def api_get(self, endpoint, *args, **kwargs):
    # 隱私模式：comma.ai API 請求都不送出（prime 狀態、firehose、行程統計…）
    # 例外：裝置註冊（v2/pilotauth），只在本機沒有任何裝置 ID 時才會呼叫，sunnylink 需要這個 ID
    if privacy_mode() and endpoint not in PRIVACY_ALLOWED_ENDPOINTS:
      raise ConnectionError(f"PrivacyMode: comma.ai API blocked ({endpoint})")
    return super().api_get(endpoint, *args, **kwargs)
