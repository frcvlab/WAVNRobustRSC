This paper proposes a novel path-finding algorithm
for wide-area, image-guided navigation. In the Wide Area Vi-
sual Navigation (WAVN) approach, a robot-centric blockchain
consensus mechanism is used to share visual information. We
show that the proposed approach improves on the prior graph-
based approach and on an RRT* based approach to WAVN path
planning.

We develop a novel navigation algorithm, Randomly Shortened
Chain (RSC), with linear computational complexity, and a robust
RSC version that can handle landmark visibility loss (RRSC). We
present numerical simulation results comparing RSC, the prior
graph-based approach, and the well-known RRT* approach for
WAVN planning. Both RSC and RRT* are significantly faster
than the graph-based approach but at the cost of longer paths.
For RSC the average path is within 1.8 times the optimal,
graph-based result. RSC improves on RRT* in both average
path length and computation time, because it leverages the
WAVN ledger information. When the robust versions of RSC
and the graph-based approach are compared, RRSC shows a 72%
improvement in reaching a target despite visibility failures during
path execution while preserving its computational advantage - an
essential plus for real-world environments.
