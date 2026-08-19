import sys
from pathlib import Path


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'LinearKalmanFilter_Implementation'))
sys.path.insert(0, str(ROOT / 'LinearKalmanFilter_Implementation' / 'LinearKF_Pendelum'))
