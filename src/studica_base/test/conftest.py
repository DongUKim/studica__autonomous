# ROS 없이 순수 로직 모듈만 임포트하기 위한 경로 설정
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
