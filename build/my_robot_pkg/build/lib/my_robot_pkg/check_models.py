import google.generativeai as genai

GEMINI_API_KEY = "본인의 API 키"
genai.configure(api_key=GEMINI_API_KEY)

print("--- [내 계정에서 사용 가능한 모델 목록] ---")
try:
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            print(f"- {m.name}")
except Exception as e:
    print(f"목록 조회 실패: {e}")