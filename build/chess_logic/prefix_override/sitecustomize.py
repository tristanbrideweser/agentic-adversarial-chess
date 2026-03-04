import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/k/GradSchool/Spring2026/CS593/fr3-embodied-ai-chess/install/chess_logic'
