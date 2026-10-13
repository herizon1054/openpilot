using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct ControlsStateExt @0x81c2f05a394cf4af {
  alkaActive @0 :Bool;
}

struct CarStateExt @0xaedffd8f31e7b55d {
  # dp - ALKA: lkasOn state from carstate (mirrors panda's lkas_on)
  lkasOn @0 :Bool;
  # dp - 豐田自動煞車維持 (Auto Brake Hold) 正在保持中
  autoBrakeHoldActive @1 :Bool;
}

struct ModelExt @0xf35cc4560bbf6ec2 {
  leftEdgeDetected @0 :Bool;
  rightEdgeDetected @1 :Bool;
}

struct DashyState @0xda96579883444c35 {
  # Pre-serialized JSON bytes for dashy UI
  # Aggregates all topics needed by dashy into single message
  json @0 :Data;
}
struct LongitudinalPlanDP @0x80ae746ee2596b11 {
  accelPersonality @0 :AccelerationPersonality;
  longitudinalPlanSource @1 :LongitudinalPlanSource;
  vTarget @2 :Float32;
  aTarget @3 :Float32;
  targets @4 :List(Target);

  struct Target {
    available @0: Bool;
    enable @1: Bool;
    action @2: Bool;
    braking @3: Bool;
    vTarget @4 :Float32;
    aTarget @5 :Float32;
    outputVtarget @6 :Float32;
    outputAtarget @7 :Float32;
  }

  enum LongitudinalPlanSource {
    cruise @0;
    dtsc @1;
    trafficStop @2;
  }
  enum AccelerationPersonality {
    sport @0;
    normal @1;
    eco @2;
  }
}
struct LiveGPS @0xa5cd762cd951a455 {
  # Position
  latitude @0 :Float64;                # degrees
  longitude @1 :Float64;               # degrees
  altitude @2 :Float64;                # meters (WGS84)

  # Motion
  speed @3 :Float32;                   # m/s (horizontal speed)
  bearingDeg @4 :Float32;              # degrees (heading)

  # Accuracy
  horizontalAccuracy @5 :Float32;      # meters
  verticalAccuracy @6 :Float32;        # meters

  # Status
  gpsOK @7 :Bool;                      # livePose valid + GPS fresh
  status @8 :Status;

  enum Status {
    uninitialized @0;    # no GPS data yet
    uncalibrated @1;     # has GPS but fusion not ready (raw passthrough)
    valid @2;            # fusion active with calibrated bearing
  }

  # Metadata
  unixTimestampMillis @9 :Int64;
  lastGpsTimestamp @10 :UInt64;        # logMonoTime of last GPS

  # livePose health (for debugging fusion issues)
  livePoseOk @11 :Bool;                # livePose valid and providing orientation/velocity
}
# ==========================================
# TDX 高公局路況預警系統資料結構
# ==========================================
# 佔用 CustomReserved6 槽位（Event @113），ID 沿用原保留 ID
struct Tdx @0xf98d843bfd7004a3 {
  sectionId @0 :Text;           # 目前路段 SectionID，空字串 = 未匹配
  speed @1 :Int16;              # 目前路段旅行車速 (km/h)，-1 = 無資料
  nextSectionId @2 :Text;       # 前方約 3 km 處所在路段 SectionID
  nextSpeed @3 :Int16;          # 前方路段旅行車速 (km/h)，-1 = 無資料
  congestion @4 :Congestion;    # 依 nextSpeed 分級
  events @5 :List(Event);       # 本週期要顯示的事件（目前/前方每 10 秒輪替，空 = 無事件）

  struct Event {
    eventType @0 :UInt8;        # 高公局 LiveEvent EventType，無法解析時為 0
    description @1 :Text;
    isCurrentSection @2 :Bool;  # true = 目前路段，false = 前方
  }

  enum Congestion {
    unknown @0;    # 無車速資料
    smooth @1;     # >= 80 km/h
    slow @2;       # 40 ~ 79 km/h
    congested @3;  # < 40 km/h
  }
}
# ==========================================

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
