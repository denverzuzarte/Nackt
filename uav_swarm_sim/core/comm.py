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


# ---------------------------------------------------------------------------
# Section 5.1 RF Propagation & Multi-Hop Path Evaluation (IEEE 802.11ax)
# ---------------------------------------------------------------------------
def path_loss(d: float) -> float:
    """
    Path loss in dB according to Report Section 5.1:
    PL(d) = PL(d_0) + 10 * n * log10(d / d_0)
    """
    d_eff = max(float(d), cfg.RF_D0)
    return cfg.RF_REFERENCE_LOSS_DB + 10.0 * cfg.RF_PATH_LOSS_EXPONENT * np.log10(d_eff / cfg.RF_D0)


def received_power(d: float) -> float:
    """
    Received power in dBm according to Report Section 5.1:
    P_rx = P_tx + G_tx + G_rx - PL(d)
    """
    if float(d) > cfg.COMM_RANGE:
        return -float("inf")
    return cfg.RF_TX_POWER_DBM + cfg.RF_TX_GAIN_DBI + cfg.RF_RX_GAIN_DBI - path_loss(d)


def link_snr(d: float) -> float:
    """Signal-to-Noise Ratio in dB."""
    prx = received_power(d)
    if prx == -float("inf"):
        return -float("inf")
    return prx - cfg.RF_NOISE_FLOOR_DBM


def hop_latency_ms(d: float, pkt_size_bytes: int = 128) -> float:
    """
    Per-hop transmission latency in ms over IEEE 802.11ax at 2.4 GHz.
    Calibrated against NS-3 discrete-event simulations (~0.35 ms base).
    """
    base_ms = 0.32 + (pkt_size_bytes / 1024.0) * 0.15
    jitter_ms = (float(d) / cfg.COMM_RANGE) * 0.05
    return base_ms + jitter_ms


def find_path_to_gcs(nodes: dict, start_node) -> list | None:
    """
    Returns shortest path [start_node, ..., GCS] or None if unreachable.
    """
    if start_node == "GCS":
        return ["GCS"]
    adj = build_graph(nodes)
    queue = deque([[start_node]])
    visited = {start_node}
    while queue:
        path = queue.popleft()
        node = path[-1]
        if node == "GCS":
            return path
        for nbr in adj.get(node, []):
            if nbr not in visited:
                visited.add(nbr)
                queue.append(path + [nbr])
    return None


def evaluate_path_metrics(nodes: dict, path: list, pkt_size_bytes: int = 128):
    """
    Evaluates multi-hop RF metrics and latency along a concrete path to GCS.
    Returns (delivered, latency_ms, hops, path_losses, snrs)
    """
    if not path or len(path) < 2:
        return False, None, 0, [], []

    hops = len(path) - 1
    total_latency_ms = 0.0
    pls = []
    snrs = []
    path_success_prob = 1.0

    for i in range(hops):
        u, v = path[i], path[i + 1]
        d = float(np.linalg.norm(nodes[u] - nodes[v]))
        if d >= cfg.COMM_RANGE:
            return False, None, hops, pls, snrs
        pl = path_loss(d)
        snr = link_snr(d)
        pls.append(pl)
        snrs.append(snr)
        total_latency_ms += hop_latency_ms(d, pkt_size_bytes)
        # Link success probability under MCS0 given high SNR (>30 dB)
        hop_per = 1e-4 if snr > 20.0 else 0.05
        path_success_prob *= (1.0 - hop_per)

    delivered = path_success_prob >= 0.95
    return delivered, total_latency_ms, hops, pls, snrs
