import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('pose_input2.csv')
df.columns = df.columns.str.strip()

plt.figure(figsize=(12, 5))

# 위치 플롯
plt.subplot(121)
plt.scatter(df['x_gls'], df['y_gls'], c='blue', label='GLS', s=50)
plt.scatter(df['x_vslam'], df['y_vslam'], c='red', label='VSLAM', s=50)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.title('Positions')
plt.axis('equal')
plt.grid(True)

# 헤딩 플롯
plt.subplot(122)
plt.scatter(range(len(df)), df['theta_gls'], label='GLS heading', s=50)
plt.scatter(range(len(df)), df['theta_vslam'], label='VSLAM heading', s=50)
plt.xlabel('Sample')
plt.ylabel('Heading (deg)')
plt.legend()
plt.title('Headings')
plt.grid(True)

plt.tight_layout()
plt.savefig('coordinate_check.png', dpi=150)
plt.show()