import itertools as it

from bisect import bisect_left
from typing import List

import numpy as np
import pandas as pd
import scipy.stats as ss

from pandas import Categorical


def VD_A(treatment: List[float], control: List[float]):
    """
    Computes Vargha and Delaney A index
    A. Vargha and H. D. Delaney.
    A critique and improvement of the CL common language
    effect size statistics of McGraw and Wong.
    Journal of Educational and Behavioral Statistics, 25(2):101-132, 2000
    The formula to compute A has been transformed to minimize accuracy errors
    See: http://mtorchiano.wordpress.com/2014/05/19/effect-size-of-r-precision/
    :param treatment: a numeric list
    :param control: another numeric list
    :returns the value estimate and the magnitude
    """
    m = len(treatment)
    n = len(control)

    if m != n:
        raise ValueError("Data d and f must have the same length")

    r = ss.rankdata(treatment + control)
    r1 = sum(r[0:m])

    # Compute the measure
    # A = (r1/m - (m+1)/2)/n # formula (14) in Vargha and Delaney, 2000
    A = (2 * r1 - m * (m + 1)) / (2 * n * m)  # equivalent formula to avoid accuracy errors

    levels = [0.147, 0.33, 0.474]  # effect sizes from Hess and Kromrey, 2004
    magnitude = ["negligible", "small", "medium", "large"]
    scaled_A = (A - 0.5) * 2

    magnitude = magnitude[bisect_left(levels, abs(scaled_A))]
    estimate = A

    return estimate, magnitude


def VD_A_DF(data, val_col: str = None, group_col: str = None, sort=True):
    """
    :param data: pandas DataFrame object
        An array, any object exposing the array interface or a pandas DataFrame.
        Array must be two-dimensional. Second dimension may vary,
        i.e. groups may have different lengths.
    :param val_col: str, optional
        Must be specified if `a` is a pandas DataFrame object.
        Name of the column that contains values.
    :param group_col: str, optional
        Must be specified if `a` is a pandas DataFrame object.
        Name of the column that contains group names.
    :param sort : bool, optional
        Specifies whether to sort DataFrame by group_col or not. Recommended
        unless you sort your data manually.
    :return: stats : pandas DataFrame of effect sizes
    Stats summary ::
    'A' : Name of first measurement
    'B' : Name of second measurement
    'estimate' : effect sizes
    'magnitude' : magnitude
    """

    x = data.copy()
    if sort:
        x[group_col] = Categorical(x[group_col], categories=x[group_col].unique(), ordered=True)
        x.sort_values(by=[group_col, val_col], ascending=True, inplace=True)

    groups = x[group_col].unique()

    # Pairwise combinations
    g1, g2 = np.array(list(it.combinations(np.arange(groups.size), 2))).T

    # Compute effect size for each combination
    ef = np.array([VD_A(list(x[val_col][x[group_col] == groups[i]].values),
                        list(x[val_col][x[group_col] == groups[j]].values)) for i, j in zip(g1, g2)])

    return pd.DataFrame({
        'A': np.unique(data[group_col])[g1],
        'B': np.unique(data[group_col])[g2],
        'estimate': ef[:, 0],
        'magnitude': ef[:, 1]
    })


if __name__ == '__main__':
    # -------------------------Import Data------------------------------------------------------------------------

    # Import Data for Prioritized, CBS and Individual(Distributed) Planners for medium or high demand
    df1 = pd.read_excel('Data_Prioritized_high.xlsx', sheet_name='Data Prioritized', engine='openpyxl')
    df2 = pd.read_excel('Data_CBS_high.xlsx', sheet_name='Data CBS', engine='openpyxl')
    df3 = pd.read_excel('Data_Individual_high.xlsx', sheet_name='Data Individual', engine='openpyxl')

    # List of Key performance indicators for Prioritized
    Mean_taxitime_prioritized = df1['Mean Taxitime'].tolist()
    Variance_taxitime_prioritized = df1['Variance Taxitime'].tolist()
    Variance_cost_prioritized = df1['Variance Cost'].tolist()
    CPU_prioritized = df1['CPU Time'].tolist()

    # List of Key performance indicators for CBS
    Mean_taxitime_cbs = df2['Mean Taxitime'].tolist()
    Variance_taxitime_csb = df2['Variance Taxitime'].tolist()
    Variance_cost_cbs = df2['Variance Cost'].tolist()
    CPU_cbs = df2['CPU Time'].tolist()

    # List of Key performance indicators for Individual Planner
    Mean_taxitime_individual = df3['Mean Taxitime'].tolist()
    Variance_taxitime_individual = df3['Variance Taxitime'].tolist()
    Variance_cost_individual = df3['Variance Cost'].tolist()
    CPU_individual = df3['CPU Time'].tolist()

    # ------------------------Computes Vargha and Delaney A index------------------------------------------

    # Comparison between Prioritized and CBS
    Mean_taxitime12 = VD_A(Mean_taxitime_prioritized, Mean_taxitime_cbs)
    Variance_taxitime12 = VD_A(Variance_taxitime_prioritized, Variance_taxitime_csb)
    Variance_cost12 = VD_A(Variance_cost_prioritized, Variance_cost_cbs)
    Cpu12 = VD_A(CPU_prioritized, CPU_cbs)

    print('Comparison between Prioritized and CBS')
    print('Mean Taxitime', Mean_taxitime12)
    print('Variance Taxitime', Variance_taxitime12)
    print('Variance Cost', Variance_cost12)
    print('CPU time', Cpu12)
    print()

    # Comparison between Prioritized and Individual
    Mean_taxitime13 = VD_A(Mean_taxitime_prioritized, Mean_taxitime_individual)
    Variance_taxitime13 = VD_A(Variance_taxitime_prioritized, Variance_taxitime_individual)
    Variance_cost13 = VD_A(Variance_cost_prioritized, Variance_cost_individual)
    Cpu13 = VD_A(CPU_prioritized, CPU_individual)

    print('Comparison between Prioritized and Individual')
    print('Mean Taxitime', Mean_taxitime13)
    print('Variance Taxitime', Variance_taxitime13)
    print('Variance Cost', Variance_cost13)
    print('CPU time', Cpu13)
    print()

    # Comparison between CBS and Individual
    Mean_taxitime23 = VD_A(Mean_taxitime_cbs, Mean_taxitime_individual)
    Variance_taxitime23 = VD_A(Variance_taxitime_csb, Variance_taxitime_individual)
    Variance_cost23 = VD_A(Variance_cost_cbs, Variance_cost_individual)
    Cpu23 = VD_A(CPU_cbs, CPU_individual)

    print('Comparison between CBS and Individual')
    print('Mean Taxitime', Mean_taxitime23)
    print('Variance Taxitime', Variance_taxitime23)
    print('Variance Cost', Variance_cost23)
    print('CPU time', Cpu23)
    print()


