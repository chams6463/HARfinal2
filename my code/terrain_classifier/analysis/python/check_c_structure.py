"""
check_c_structure.py
===============================================================================
Locomotion_Mode_Classifier.c 의 구조 정합성 점검.

ARM 툴체인(arm-none-eabi-gcc)과 호스트 gcc 가 모두 없는 환경이라 실제 컴파일을
할 수 없다. 컴파일러가 잡아줄 오류 중 기계적으로 확인 가능한 것만 검사한다:
  - 괄호 균형
  - static 함수 프로토타입 <-> 정의 일대일 대응
  - TSM 콜백 시그니처
  - 미사용 static 변수
  - 필수 심볼 존재

주의: 이것은 컴파일 검증이 아니다. 타입 오류/미선언 식별자 등은 잡지 못한다.
===============================================================================
"""

import io
import re
import sys

PATH = r"D:\ifac_wearable_contest\XM_Apps\User_Algorithm\Locomotion_Mode_Classifier.c"

BACKSLASH = chr(92)
QUOTE = chr(34)


def strip_comments_and_strings(src):
    """주석과 문자열/문자 리터럴을 제거한 코드만 남긴다."""
    t = re.sub(r"/[*].*?[*]/", "", src, flags=re.S)
    t = re.sub(r"//[^\n]*", "", t)

    out = []
    i, n = 0, len(t)
    while i < n:
        c = t[i]
        if c == QUOTE or c == "'":
            quote = c
            i += 1
            while i < n and t[i] != quote:
                i += 2 if t[i] == BACKSLASH else 1
            i += 1
            out.append("0")
        else:
            out.append(c)
            i += 1
    return "".join(out)


def main():
    src = io.open(PATH, encoding="utf-8").read()
    code = strip_comments_and_strings(src)
    fails = []

    print("=" * 66)
    print(" Locomotion_Mode_Classifier.c 구조 점검")
    print(" (주의: 실제 컴파일이 아님 — 툴체인 부재. 기계적 검사만.)")
    print("=" * 66)

    print("\n[1] 괄호 균형")
    for op, cl, name in [("{", "}", "중괄호"), ("(", ")", "소괄호"), ("[", "]", "대괄호")]:
        a, b = code.count(op), code.count(cl)
        ok = a == b
        if not ok:
            fails.append("괄호 불균형: " + name)
        print("    %-8s %4d / %-4d  %s" % (name, a, b, "OK" if ok else "FAIL"))

    print("\n[2] static 함수 프로토타입 <-> 정의")
    # 이름은 단어 경계로 잡아야 한다(Active_Entry 를 _Entry 로 자르면 안 됨).
    # 정의는 ')' 다음 줄에 '{' 가 오므로 개행을 허용해야 한다.
    NAME = r"\b([A-Za-z_][A-Za-z0-9_]*)"
    protos = set(re.findall(r"^static [^;{()]*?" + NAME + r"\s*\([^;]*\);", code, re.M))
    defs = set(re.findall(r"^static [^;{()]*?" + NAME + r"\s*\([^;{]*\)\s*\{", code, re.M))
    print("    프로토타입 %d개, 정의 %d개" % (len(protos), len(defs)))
    missing_def = sorted(protos - defs)
    missing_proto = sorted(defs - protos)
    if missing_def:
        fails.append("정의 없는 프로토타입")
    if missing_proto:
        fails.append("프로토타입 없는 정의")
    print("    정의 없는 프로토타입: %s" % (missing_def or "없음"))
    print("    프로토타입 없는 정의: %s" % (missing_proto or "없음"))

    print("\n[3] TSM 콜백 시그니처")
    for f in ["Off_Loop", "Standby_Loop", "Active_Entry", "Active_Loop", "Active_Exit"]:
        ok = re.search(r"static void " + f + r"\s*\(void\)\s*\{", code) is not None
        if not ok:
            fails.append("TSM 콜백 누락: " + f)
        print("    %-14s %s" % (f, "OK" if ok else "FAIL"))

    print("\n[4] 미사용 static 변수")
    unused = []
    for m in re.finditer(r"^static [^;=]*?(s_[A-Za-z0-9_]+)", code, re.M):
        v = m.group(1)
        if len(re.findall(r"\b" + v + r"\b", code)) < 2:
            unused.append(v)
    print("    %s" % (sorted(set(unused)) or "없음"))
    if unused:
        fails.append("미사용 static 변수")

    print("\n[5] 필수 심볼")
    checks = [
        ("void User_Setup(void)", code, "User_Setup 정의"),
        ("void User_Loop(void)", code, "User_Loop 정의"),
        ("XM_IO_Update()", code, "XM_IO_Update 호출(버튼 필수)"),
        ("ref_curve_tables.h", src, "ref_curve_tables.h include"),
        ("XM_SwitchDioToAdc", code, "외부 ADC 활성화"),
        ("MUSCLE_SITE_COUNT", src, "EMG 자리 수 상수화"),
    ]
    for needle, hay, label in checks:
        ok = needle in hay
        if not ok:
            fails.append(label)
        print("    %-32s %s" % (label, "OK" if ok else "FAIL"))

    print("\n[6] 제거되어야 할 잔재")
    for needle, label in [("MODE_REJECT_TH", "미사용 상수 MODE_REJECT_TH")]:
        ok = needle not in src
        if not ok:
            fails.append(label)
        print("    %-32s %s" % (label, "없음(OK)" if ok else "있음(FAIL)"))

    print("\n" + "=" * 66)
    if fails:
        print(" 결과: FAIL — %s" % ", ".join(fails))
        return 1
    print(" 결과: 기계적 검사 전부 통과")
    print(" (미검증: 타입 정합성, 미선언 식별자 — 컴파일러 필요)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
