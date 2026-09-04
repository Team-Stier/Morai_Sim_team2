# -*- coding: utf-8 -*-
"""
nmea_gps.py
- 역할: MORAI GPS UDP로 들어오는 NMEA 문장($GPGGA, $GPRMC)을 파싱해
        위경도/고도/fix 상태를 담은 순수 데이터 구조로 변환한다.
        ROS에 의존하지 않으므로 단위 테스트가 가능하다.
- 주요 구조체/함수: GpsFix, parse_nmea_sentence, build_gga, build_rmc

NMEA 참고: https://ko.wikipedia.org/wiki/NMEA_0183
좌표는 ddmm.mmmm(위도) / dddmm.mmmm(경도) 형식이며 N/S, E/W 방향으로 부호를 정한다.
"""

# NavSatStatus.status 값과 동일하게 맞춘 상수(파서는 ROS에 의존하지 않도록 직접 정의).
STATUS_NO_FIX = -1
STATUS_FIX = 0
STATUS_SBAS_FIX = 1
STATUS_GBAS_FIX = 2


class NmeaParseError(Exception):
    """NMEA 문장을 해석할 수 없을 때 발생한다."""
    pass


class GpsFix(object):
    """파싱된 GPS 위치 한 건. altitude는 GGA에만 존재하므로 RMC에서는 None."""

    __slots__ = ("sentence_type", "latitude", "longitude", "altitude", "status")

    def __init__(self, sentence_type, latitude, longitude, altitude, status):
        self.sentence_type = sentence_type  # 'GGA' 또는 'RMC'
        self.latitude = latitude            # 부호 반영된 십진 도, 값 없으면 None
        self.longitude = longitude          # 부호 반영된 십진 도, 값 없으면 None
        self.altitude = altitude            # m, GGA만, 없으면 None
        self.status = status                # STATUS_* 중 하나

    @property
    def has_position(self):
        return self.latitude is not None and self.longitude is not None

    def __repr__(self):
        return ("GpsFix(type=%s, lat=%r, lon=%r, alt=%r, status=%d)"
                % (self.sentence_type, self.latitude, self.longitude,
                   self.altitude, self.status))

    def __eq__(self, other):
        if not isinstance(other, GpsFix):
            return NotImplemented
        return all(getattr(self, s) == getattr(other, s) for s in self.__slots__)


# 함수이름: nmea_checksum
# 기능: '$'와 '*' 사이 payload의 XOR checksum을 계산한다.
# 인자: payload - '$'와 '*'를 제외한 문장 본문 문자열
# 반환값: 0~255 정수 checksum
def nmea_checksum(payload):
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    return checksum & 0xFF


# 함수이름: format_sentence
# 기능: payload에 '$'와 '*체크섬'을 붙여 완전한 NMEA 문장을 만든다.
# 인자: payload - 본문(예: "GPGGA,...")
# 반환값: 완성된 NMEA 문장 문자열
def format_sentence(payload):
    return "$%s*%02X" % (payload, nmea_checksum(payload))


# 함수이름: _parse_float
# 기능: 빈 문자열은 None, 나머지는 float 변환. 실패 시 예외.
def _parse_float(text):
    text = text.strip()
    if text == "":
        return None
    try:
        return float(text)
    except ValueError:
        raise NmeaParseError("float 변환 실패: %r" % text)


# 함수이름: _ddmm_to_degrees
# 기능: ddmm.mmmm(또는 dddmm.mmmm) 문자열과 방향을 십진 도로 변환한다.
# 인자: raw - 좌표 문자열, direction - 'N'/'S'/'E'/'W'
# 반환값: 부호 반영 십진 도(float), 좌표 필드가 비어 있으면 None
def _ddmm_to_degrees(raw, direction):
    raw = raw.strip()
    direction = direction.strip().upper()
    if raw == "":
        return None
    try:
        value = float(raw)
    except ValueError:
        raise NmeaParseError("좌표 float 변환 실패: %r" % raw)
    degrees = int(value // 100)
    minutes = value - degrees * 100.0
    if minutes < 0.0 or minutes >= 60.0:
        raise NmeaParseError("분(minutes) 범위 오류: %r" % raw)
    result = degrees + minutes / 60.0
    if direction in ("S", "W"):
        result = -result
    elif direction not in ("N", "E"):
        raise NmeaParseError("방향 값 오류: %r" % direction)
    return result


# 함수이름: _gga_quality_to_status
# 기능: GGA quality 필드를 NavSatStatus.status 값으로 변환한다.
def _gga_quality_to_status(quality_text):
    quality_text = quality_text.strip()
    if quality_text == "" or quality_text == "0":
        return STATUS_NO_FIX
    try:
        quality = int(quality_text)
    except ValueError:
        raise NmeaParseError("GGA quality 정수 변환 실패: %r" % quality_text)
    if quality <= 0:
        return STATUS_NO_FIX
    if quality == 1:
        return STATUS_FIX
    # 2=DGPS, 4/5=RTK 등 보정 fix는 지상 보정으로 취급한다.
    return STATUS_GBAS_FIX


# 함수이름: _rmc_status_to_status
# 기능: RMC posStatus(A/V)를 NavSatStatus.status 값으로 변환한다.
def _rmc_status_to_status(status_text):
    status_text = status_text.strip().upper()
    if status_text == "A":
        return STATUS_FIX
    return STATUS_NO_FIX


# 함수이름: _parse_gga
# 기능: GGA 필드 리스트를 GpsFix로 변환한다.
def _parse_gga(fields):
    # 0:GPGGA 1:utc 2:lat 3:N/S 4:lon 5:E/W 6:quality 7:sats 8:hdop 9:alt 10:M ...
    if len(fields) < 10:
        raise NmeaParseError("GGA 필드 부족: %d" % len(fields))
    latitude = _ddmm_to_degrees(fields[2], fields[3])
    longitude = _ddmm_to_degrees(fields[4], fields[5])
    status = _gga_quality_to_status(fields[6])
    altitude = _parse_float(fields[9])
    if latitude is None or longitude is None:
        status = STATUS_NO_FIX
    return GpsFix("GGA", latitude, longitude, altitude, status)


# 함수이름: _parse_rmc
# 기능: RMC 필드 리스트를 GpsFix로 변환한다.
def _parse_rmc(fields):
    # 0:GPRMC 1:utc 2:status 3:lat 4:N/S 5:lon 6:E/W 7:speed 8:track 9:date ...
    if len(fields) < 7:
        raise NmeaParseError("RMC 필드 부족: %d" % len(fields))
    latitude = _ddmm_to_degrees(fields[3], fields[4])
    longitude = _ddmm_to_degrees(fields[5], fields[6])
    status = _rmc_status_to_status(fields[2])
    if latitude is None or longitude is None:
        status = STATUS_NO_FIX
    return GpsFix("RMC", latitude, longitude, None, status)


# 함수이름: parse_nmea_sentence
# 기능: 한 개의 NMEA 문장 문자열을 GpsFix로 파싱한다.
# 인자: raw - NMEA 문장, require_checksum - '*'checksum 검증 여부
# 반환값: GpsFix
# 예외: NmeaParseError (형식 오류, checksum 불일치, 미지원 문장 등)
def parse_nmea_sentence(raw, require_checksum=True):
    if raw is None:
        raise NmeaParseError("빈 입력")
    sentence = raw.strip().strip("\x00")
    if not sentence.startswith("$"):
        raise NmeaParseError("'$'로 시작하지 않음")

    if "*" in sentence:
        payload, _, checksum_text = sentence[1:].partition("*")
        checksum_text = checksum_text.strip()
        if len(checksum_text) < 2:
            raise NmeaParseError("checksum 자릿수 부족")
        try:
            expected = int(checksum_text[:2], 16)
        except ValueError:
            raise NmeaParseError("checksum 16진 변환 실패: %r" % checksum_text)
        if nmea_checksum(payload) != expected:
            raise NmeaParseError("checksum 불일치")
    else:
        if require_checksum:
            raise NmeaParseError("checksum 없음")
        payload = sentence[1:]

    fields = payload.split(",")
    talker_type = fields[0]
    sentence_type = talker_type[2:5] if len(talker_type) >= 5 else talker_type
    if sentence_type == "GGA":
        return _parse_gga(fields)
    if sentence_type == "RMC":
        return _parse_rmc(fields)
    raise NmeaParseError("미지원 문장: %r" % talker_type)


# ---------------------------------------------------------------------------
# 아래는 테스트/가짜 송신기가 재사용하는 NMEA 문장 생성 헬퍼(파서와 대칭).
# ---------------------------------------------------------------------------

# 함수이름: _degrees_to_ddmm
# 기능: 십진 도를 ddmm.mmmm 문자열과 방향 문자로 변환한다.
def _degrees_to_ddmm(value, is_longitude):
    if is_longitude:
        hemisphere = "E" if value >= 0.0 else "W"
        degree_width = 3
    else:
        hemisphere = "N" if value >= 0.0 else "S"
        degree_width = 2
    magnitude = abs(value)
    degrees = int(magnitude)
    minutes = (magnitude - degrees) * 60.0
    coordinate = "%0*d%07.4f" % (degree_width, degrees, minutes)
    return coordinate, hemisphere


# 함수이름: build_gga
# 기능: 위경도/고도/quality로 유효한 $GPGGA 문장을 만든다(테스트/모의 송신용).
def build_gga(latitude, longitude, altitude, quality=1, utc="123519.00",
              num_sats=8, hdop=0.9):
    lat_str, lat_dir = _degrees_to_ddmm(latitude, False)
    lon_str, lon_dir = _degrees_to_ddmm(longitude, True)
    payload = ("GPGGA,%s,%s,%s,%s,%s,%d,%02d,%.1f,%.1f,M,0.0,M,,"
               % (utc, lat_str, lat_dir, lon_str, lon_dir, quality,
                  num_sats, hdop, altitude))
    return format_sentence(payload)


# 함수이름: build_rmc
# 기능: 위경도/상태로 유효한 $GPRMC 문장을 만든다(테스트/모의 송신용).
def build_rmc(latitude, longitude, status="A", utc="123519.00",
              speed_knots=0.0, track_deg=0.0, date="230725"):
    lat_str, lat_dir = _degrees_to_ddmm(latitude, False)
    lon_str, lon_dir = _degrees_to_ddmm(longitude, True)
    payload = ("GPRMC,%s,%s,%s,%s,%s,%s,%.1f,%.1f,%s,,,A"
               % (utc, status, lat_str, lat_dir, lon_str, lon_dir,
                  speed_knots, track_deg, date))
    return format_sentence(payload)

