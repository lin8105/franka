import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def plot_citr_fingerprint(csv_file):
    # 1. Read data and drop the last potentially incomplete row containing NaNs
    df = pd.read_csv(csv_file).dropna()
    
    # 2. Extract 10-dimensional CITR features
    citr_cols = [
        'citr_ff', 'citr_ftau', 'citr_tautau',
        'citr_fv', 'citr_tauv', 'citr_vv',
        'citr_fw', 'citr_tauw', 'citr_vw', 'citr_ww'
    ]
    
    # Extract data and transpose: make Y-axis represent 10 features and X-axis represent time series N
    citr_data = df[citr_cols].values.T 

    # 3. Core algorithm: Perform row-wise unbiased scaling according to paper equation (11)
    # Calculate the maximum absolute value for each row (each feature)
    max_abs_vals = np.max(np.abs(citr_data), axis=1, keepdims=True)
    
    # Avoid division by zero (for static features that remain consistently 0)
    max_abs_vals[max_abs_vals == 0] = 1e-6
    
    # Normalize to the range [-1, 1]
    citr_normalized = citr_data / max_abs_vals

    # 4. Plot the heatmap (Task Fingerprint)
    plt.figure(figsize=(10, 6))
    
    # The paper recommends using a divergent colormap that distinguishes positive and negative values, such as 'jet' or 'coolwarm'
    ax = sns.heatmap(citr_normalized, cmap='jet', center=0, vmin=-1, vmax=1, cbar=True)

    feature_labels = [
        r'$\langle f, f \rangle$', r'$\langle f, \tau \rangle$', r'$\langle \tau, \tau \rangle$',
        r'$\langle f, v \rangle$', r'$\langle \tau, v \rangle$', r'$\langle v, v \rangle$',
        r'$\langle f, \omega \rangle$', r'$\langle \tau, \omega \rangle$', r'$\langle v, \omega \rangle$', r'$\langle \omega, \omega \rangle$'
    ]
    
    plt.yticks(ticks=np.arange(10) + 0.5, labels=feature_labels, rotation=0, fontsize=14)
    plt.xticks([])  
    
    # Add time arrow indicator on X-axis
    plt.xlabel(r'$\rightarrow t$', fontsize=16, loc='left')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_citr_fingerprint('./robot_states.csv')