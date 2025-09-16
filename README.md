# Timetable Nodes for Public Transport Network 
Realization of the paper ["Timetable Nodes for Public Transport Network"](https://doi.org/10.48550/arXiv.2410.15715)
# Example of running 

## Algorithms:

This repo contains Python realization of the following pathfinding algorithms:
- [Connection Scan Algorithm (CSA)](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/algorithms/connection_scan_algorithm.py)
- [RAPTOR](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/algorithms/raptor.py)
- [TD-Dijkstra](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/algorithms/dijkstra.py)
- [Contraction Hierarchy](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/data_structures/graph.py)
- [Forward Search](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/algorithms/forward_search.py)
- [Timetable Nodes](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/algorithms/dijkstra.py)

## Data:
For the reproducibility of the results, all data can be found via the following links:
- [Datasets of 25 cities in the experiment](https://unsw-my.sharepoint.com/:f:/g/personal/z5439725_ad_unsw_edu_au/EhybYPEkE-pOlbne7-Bu238BQNVd8Zee1ppydW3UgfSiZA?e=ZZ5gNp)
- [All the precalculated datastructures](https://unsw-my.sharepoint.com/:f:/g/personal/z5439725_ad_unsw_edu_au/EtvFl8qnXe5InN5jAq0TmJgBHi0ZjYLe06UPFWyXSwwLzw?e=YsVbPg)

## Notbooks:
- [Calculations of the transitive closure for CSA and RAPTOR](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/notebooks/Calculate_transitive_clousure.ipynb)
- [CH experiment from paper "Timetable Nodes for Public Transport Network"](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/notebooks/Run_pathfinding_contraction_hierarchy_experiment.ipynb)
- [Experiment from paper "Multimodal Pathfinding with Personalized Travel Speed and Transfers of Unlimited Distance"](https://github.com/andrii-rohovyi/timetable_nodes/blob/main/notebooks/Run_pathfinding_customized_speed_of_unlimited_distance.ipynb)

# Citation
Please cite the original paper if you find this codebase useful in your research.

CH experiment:
```latex

@misc{author2024title,
  author       = {Andrii Rohovyi and Peter J. Stuckey and Toby Walsh},
  title        = {Timetable Nodes for Public Transport Network},
  year         = {2024},
  archivePrefix= {arXiv},
  eprint       = {arXiv:2410.15715},
  primaryClass = {cs.DS}
}
```

Unlimited transfer with personalized walking speed:

#### TODO: will be published soon on ICTAI 2025

# Contact

For any questions about this repo, feel free to reach out: **a.rohovyi@unsw.edu.au**, **andrii.rohovyi@postdata.ai**