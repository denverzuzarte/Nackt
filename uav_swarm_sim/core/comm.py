"""
Communication model.

Distance-threshold link model (as used in Ponda et al. 2010/2011 and, with
a soft decay, Varadharajan et al. 2020 "Swarm Relays"):

    link(i, j) exists  <=>  distance(i, j) < COMM_RANGE

We additionally track a "quality" zone (safe / critical / break-away) purely
for logging and for the relay-chain controller's expand/retract behaviour --
it does NOT change whether a link exists, only how urgently a relay should
reposition. This mirrors the zone concept in Swarm Relays without adopting
its full exponential SNR model, which is out of scope for Stage 1 (noted in
report as a "realistic link model" future-work item, citing Ladosz et al.
and Yanmaz's air-to-ground work).

Connectivity to the GCS is computed as reachability in the undirected graph
formed by all "alive" nodes (UAVs + GCS) via BFS -- i.e. strict end-to-end
multi-hop connectivity, matching the mission's "continuous relay" requirement.
"""
from __future__ import annotations
import numpy as np
from collections import deque
from . import config as cfg


class LinkZone:
    SAFE = "safe"
    CRITICAL = "critical"
    BREAKAWAY = "breakaway"
    NONE = "none"


def link_exists(pos_a: np.ndarray, pos_b: np.ndarray) -> bool:
    return float(np.linalg.norm(pos_a - pos_b)) < cfg.COMM_RANGE


def link_zone(pos_a: np.ndarray, pos_b: np.ndarray) -> str:
    d = float(np.linalg.norm(pos_a - pos_b))
    if d >= cfg.COMM_RANGE:
        return LinkZone.NONE
    if d < cfg.COMM_SAFE:
        return LinkZone.SAFE
    if d < cfg.COMM_CRITICAL:
        return LinkZone.CRITICAL
    return LinkZone.BREAKAWAY


def build_graph(nodes: dict):
    """
    nodes: dict of {node_id: position (np.ndarray)}, must include 'GCS'.
    Returns adjacency dict {node_id: set(neighbour_ids)}.
    """
    ids = list(nodes.keys())
    adj = {i: set() for i in ids}
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            ia, ib = ids[a], ids[b]
            if link_exists(nodes[ia], nodes[ib]):
                adj[ia].add(ib)
                adj[ib].add(ia)
    return adj


def connectivity_to_gcs(nodes: dict):
    """
    nodes: dict of {node_id: position}, must include key 'GCS'.
    Returns dict {node_id: hop_count_or_None} -- None means unreachable.
    """
    adj = build_graph(nodes)
    dist = {i: None for i in nodes}
    dist['GCS'] = 0
    q = deque(['GCS'])
    while q:
        u = q.popleft()
        for v in adj[u]:
            if dist[v] is None:
                dist[v] = dist[u] + 1
                q.append(v)
    return dist, adj
