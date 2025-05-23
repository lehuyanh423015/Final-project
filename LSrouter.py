####################################################
# LSrouter.py
# Name: Đỗ Văn Dũng
# HUID: 23021492
#####################################################

from router import Router
from packet import Packet
import json
import heapq


class LSrouter(Router):
    """Link state routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        # TODO
        #   add your own class fields and initialization code here
        self.neighbors = {}  
        self.link_state_db = {}
        self.seq_num = 0  
        self.forwarding_table = {}
        
    def broadcast_link_state(self):
        self.seq_num += 1
        links = []
        for port, (neighbor, cost) in self.neighbors.items():
            links.append((neighbor, cost))
        self.link_state_db[self.addr] = {"seq": self.seq_num, "links": links}
        lsa = json.dumps({"src": self.addr, "seq": self.seq_num, "links": links})
        for port in self.neighbors:
            self.send(port, Packet(self.addr, port, False, lsa))

        self.dijkstra()

    def dijkstra(self):
        graph = {}
        for router, info in self.link_state_db.items():
            graph[router] = {}
            for neighbor, cost in info["links"]:
                graph[router][neighbor] = cost
                
        dist = {self.addr: 0}
        prev = {}
        visited = set()
        heap = [(0, self.addr)]

        while heap:
            curr_dist, curr = heapq.heappop(heap)
            if curr in visited:
                continue
            visited.add(curr)

            for neighbor in graph.get(curr, {}):
                new_dist = curr_dist + graph[curr][neighbor]
                if neighbor not in dist or new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    prev[neighbor] = curr
                    heapq.heappush(heap, (new_dist, neighbor))

        self.forwarding_table = {}
        for dest in dist:
            if dest == self.addr:
                continue
            next_hop = dest
            while prev[next_hop] != self.addr:
                next_hop = prev[next_hop]
            for port, (neighbor, _) in self.neighbors.items():
                if neighbor == next_hop:
                    self.forwarding_table[dest] = port
                    break

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        # TODO
        if packet.is_traceroute:
            # Hint: this is a normal data packet
            # If the forwarding table contains packet.dst_addr
            #   send packet based on forwarding table, e.g., self.send(port, packet)
            if packet.dst_addr in self.forwarding_table:
                out_port = self.forwarding_table[packet.dst_addr]
                self.send(out_port, packet)
        else:
            # Hint: this is a routing packet generated by your routing protocol
            # If the sequence number is higher and the received link state is different
            #   update the local copy of the link state
            #   update the forwarding table
            #   broadcast the packet to other neighbors
            lsa = json.loads(packet.content)
            src = lsa["src"]
            seq = lsa["seq"]
            links = lsa["links"]
            if src not in self.link_state_db or self.link_state_db[src]["seq"] < seq:
                self.link_state_db[src] = {"seq": seq, "links": links}
                self.dijkstra()
                for p in self.neighbors:
                    if p != port:
                        self.send(p, packet)

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # TODO
        #   update local data structures and forwarding table
        #   broadcast the new link state of this router to all neighbors
        self.neighbors[port] = (endpoint, cost)
        self.broadcast_link_state()

    def handle_remove_link(self, port):
        """Handle removed link."""
        # TODO
        #   update local data structures and forwarding table
        #   broadcast the new link state of this router to all neighbors
        if port in self.neighbors:
            del self.neighbors[port]
        self.broadcast_link_state()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            # TODO
            #   broadcast the link state of this router to all neighbors
            self.broadcast_link_state()

    def __repr__(self):
        """Representation for debugging in the network visualizer."""
        # TODO
        #   NOTE This method is for your own convenience and will not be graded
        return f"LSrouter(addr={self.addr})"
