####################################################
# LSrouter.py
# Name: Đỗ Văn Dũng
# HUID: 23021492
####################################################

from router import Router
from packet import Packet
import json
import heapq

class LSrouter(Router):
    """Link state routing protocol implementation."""

    def __init__(self, addr, heartbeat_time):
        super().__init__(addr)  # Initialize base class
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        # sequence number for our LSPs
        self.sequence = 0
        # port -> neighbor router address and cost
        self.neighbors = {}       # port -> cost
        self.port2nbr = {}        # port -> addr
        # link-state database: addr -> (seq, {neighbor: cost, ...})
        self.lsdb = {}       # forwarding table: dest_addr -> port
        self.forwarding_table = {}

    def broadcast_lsp(self):
        """Create and flood our LSP to all neighbors."""
        self.sequence += 1
        # Our LSP: include our addr, seq, and current neighbors
        lsp = {
            'origin': self.addr,
            'seq': self.sequence,
            'links': { self.port2nbr[p]: cost for p, cost in self.neighbors.items() }
        }
        content = json.dumps(lsp)
        for port in self.neighbors:
            p = Packet(kind=Packet.ROUTING,
                       src_addr=self.addr,
                       dst_addr=None,
                       content=content)
            self.send(port, p)

    def recompute_routes(self):
        """
        Run Dijkstra on current LSDB to build forwarding table.
        """
        # Build graph from lsdb
        graph = {}
        for node, (seq, links) in self.lsdb.items():
            graph[node] = links.copy()
        # Dijkstra from self.addr
        dist = {self.addr: 0}
        prev = {}
        visited = set()
        heap = [(0, self.addr)]
        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)
            for v, w in graph.get(u, {}).items():
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))
        # Build forwarding table: for each destination, find first hop
        ft = {}
        for dest in dist:
            if dest == self.addr:
                continue
            # backtrack from dest to neighbor
            curr = dest
            while prev.get(curr) != self.addr:
                curr = prev[curr]
            # curr is neighbor next hop
            # find port for neighbor curr
            for port, nbr in self.port2nbr.items():
                if nbr == curr:
                    ft[dest] = port
                    break
        self.forwarding_table = ft

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.forwarding_table:
                self.send(self.forwarding_table[dst], packet)
        else:
            # Received LSP
            lsp = json.loads(packet.content)
            origin = lsp['origin']
            seq = lsp['seq']
            links = lsp['links']
            # Check if new or newer
            old = self.lsdb.get(origin)
            if old is None or seq > old[0]:
                # update LSDB
                self.lsdb[origin] = (seq, links)
                # flood to other neighbors
                for p in self.neighbors:
                    if p != port:
                        pck = Packet(kind=Packet.ROUTING,
                                     src_addr=self.addr,
                                     dst_addr=None,
                                     content=packet.content)
                        self.send(p, pck)
                # recompute routes
                self.recompute_routes()

    def handle_new_link(self, port, endpoint, cost):
        # neighbor added
        self.neighbors[port] = cost
        self.port2nbr[port] = endpoint
        # update own LSP and flood
        self.broadcast_lsp()
        # recompute routes including self
        self.lsdb[self.addr] = (self.sequence, {self.port2nbr[p]: self.neighbors[p] for p in self.neighbors})
        self.recompute_routes()

    def handle_remove_link(self, port):
        # neighbor removed
        if port in self.neighbors:
            self.neighbors.pop(port)
            self.port2nbr.pop(port)
            # flood updated LSP
            self.broadcast_lsp()
            # update LSDB self entry
            self.lsdb[self.addr] = (self.sequence, {self.port2nbr[p]: self.neighbors[p] for p in self.neighbors})
            self.recompute_routes()

    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # periodic LSP flood
            self.broadcast_lsp()

    def __repr__(self):
        return f"LSrouter(addr={self.addr}, ft={self.forwarding_table})"