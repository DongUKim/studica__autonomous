import os
import sys

# colcon 없이도 코어 테스트가 돌도록 패키지 루트(src/studica_repeat)를 경로에 넣는다.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
