from pydantic import BaseModel

class Settings(BaseModel):
    tick_hz: float = 50.0          # execution loop frequency
    default_move_seconds: float = 2.0
    ws_broadcast_hz: float = 25.0  # websocket broadcast rate

settings = Settings()
