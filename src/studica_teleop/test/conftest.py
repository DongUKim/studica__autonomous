import os
import sys

# ROS 없는 개발 PC에서 순수 모듈(terminal/hud/teleop_state)만 임포트해 테스트한다
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
