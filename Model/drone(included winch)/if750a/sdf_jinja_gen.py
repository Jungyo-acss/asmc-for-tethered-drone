import sys
import os
import math  # ✅ math 모듈 추가
from jinja2 import Environment, FileSystemLoader

# 인자 체크: 템플릿 파일 경로와 출력 파일 경로가 필요함.
if len(sys.argv) != 3:
    print("Usage: python3 sdf_jinja_gen.py <template_path> <output_path>")
    sys.exit(1)

# 커맨드라인 인자로부터 경로를 받아옴.
template_path = sys.argv[1]
output_path = sys.argv[2]

# 템플릿 파일의 디렉토리와 파일명을 분리.
template_dir, template_file = os.path.split(template_path)
if not template_dir:
    template_dir = '.'

# Jinja2 환경 설정
env = Environment(loader=FileSystemLoader(template_dir))

# ✅ math 모듈을 Jinja2에 추가 (중요)
template = env.get_template(template_file)
sdf_output = template.render(math=math)  # ✅ math 모듈을 Jinja에서 사용할 수 있도록 전달

# 출력 파일 경로에 SDF 내용 저장
with open(output_path, "w") as f:
    f.write(sdf_output)

print(f"✅ SDF file generated: {output_path}")
