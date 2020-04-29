using System.Collections.Generic;
using UnityEngine;

// In this file you get not one, but two Dijkstra-like algorithms.
// One is a runtime pathfinding algorithm, the other is a preprocessing grid->graph algorithm.
// Watch me write and explain the whole thing here https://youtu.be/lCMJFIleAzg

public class Dijkstra : MonoBehaviour
{
    public class DijkstraGraph
    {
        public string[] nodes;
        public uint[] edges;
    }

    public class DijkstraAlgoData
    {
        public DijkstraGraph graph;
        public uint[] open_list;
        public uint open_list_size;

        public uint[] node_cost;
        public uint[] prev_node;

        public uint starting_node;
        public uint ending_node;
    }

    public const uint INVALID_NODE = 66666666;

    /*
        nodes: List of node names.
        edges: a array where every 3 elements constructs an edge. 
            First element is starting node index, second is ending node index, third is movement cost (non-negative).
    */
    public static DijkstraGraph SetupGraph(string[] nodes, uint[] edges)
    {
        var dg = new DijkstraGraph();
        dg.nodes = nodes;
        dg.edges = edges;

        return dg;
    }

    // This function takes a grid where 1s are walls and 0s are empty space
    // and turns it into a graph. It does this by propgating through the grid using
    // Dijkstra to traverse the grid.
    public static DijkstraGraph SetupGrid(int width, int height, uint[] grid)
    {
        var dg = new DijkstraGraph();
        dg.nodes = new string[width*height];
        for (int i = 0; i < (width*height); ++i)
        {
            dg.nodes[i] = "" + i;
        }

        var open_list = new uint[grid.Length];
        open_list[0] = 0;
        uint open_list_size = 1;

        List<uint> edges = new List<uint>();

        while (open_list_size != 0)
        {
            var node = open_list[open_list_size-1];
            open_list_size -= 1;

            int north = (int)node - width;
            int south = (int)node + width;

            int east = (int)node + 1;
            int west = (int)node - 1;

            if (node % width != 0)
            {
                // west is valid
                if (grid[west] == 0)
                {
                    edges.Add(node);
                    edges.Add((uint)west);
                    edges.Add(1);

                    open_list[open_list_size] = (uint)west;
                    open_list_size += 1;
                }
            }
            if (node % width != width-1)
            {
                // east is valid
                if (grid[east] == 0)
                {
                    edges.Add(node);
                    edges.Add((uint)east);
                    edges.Add(1);

                    open_list[open_list_size] = (uint)east;
                    open_list_size += 1;
                }
            }
            if (north >= 0)
            {
                // north is valid
                if (grid[north] == 0)
                {
                    edges.Add(node);
                    edges.Add((uint)north);
                    edges.Add(1);

                    open_list[open_list_size] = (uint)north;
                    open_list_size += 1;
                }
            }
            if (south < (width*height))
            {
                // south is valid
                if (grid[south] == 0)
                {
                    edges.Add(node);
                    edges.Add((uint)south);
                    edges.Add(1);

                    open_list[open_list_size] = (uint)south;
                    open_list_size += 1;
                }
            }

            grid[node] = 2;
        }

        dg.edges = edges.ToArray();
        return dg;
    }

    public static DijkstraAlgoData StartDijkstra(DijkstraGraph graph, uint starting_node, uint ending_node)
    {
        var dad = new DijkstraAlgoData();
        dad.graph = graph;
        dad.starting_node = starting_node;
        dad.ending_node = ending_node;

        dad.node_cost = new uint[graph.nodes.Length];
        dad.prev_node = new uint[graph.nodes.Length];

        for (int i = 0; i < graph.nodes.Length; ++i)
        {
            dad.node_cost[i] = 1000000000;
            dad.prev_node[i] = INVALID_NODE;
        }

        dad.node_cost[starting_node] = 0;

        dad.open_list = new uint[graph.nodes.Length];
        dad.open_list[0] = starting_node;
        dad.open_list_size = 1;

        return dad;
    }

    public static bool Process(DijkstraAlgoData dad)
    {
        var node = dad.open_list[dad.open_list_size-1];
        dad.open_list_size -= 1;

        for (int i = 0; i < dad.graph.edges.Length; i += 3)
        {
            var edge_start = dad.graph.edges[i];
            var edge_end = dad.graph.edges[i+1];
            var cost = dad.graph.edges[i+2];

            if (edge_end == node)
            {
                var v = edge_start;
                edge_start = edge_end;
                edge_end = v;
            }

            if (edge_start == node)
            {
                if (dad.node_cost[edge_end] > (dad.node_cost[node] + cost))
                {
                    dad.node_cost[edge_end] = (dad.node_cost[node] + cost);
                    dad.prev_node[edge_end] = node;

                    dad.open_list[dad.open_list_size] = edge_end;
                    dad.open_list_size += 1;
                }
            }
        }

        if (dad.open_list_size == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public static uint[] GetPath(DijkstraAlgoData dad)
    {
        var count_walker = dad.prev_node[dad.ending_node];
        int count = 0;

        while (count_walker != INVALID_NODE)
        {
            count += 1;
            count_walker = dad.prev_node[count_walker];
        }

        if (count == 0)
        {
            return null;
        }
        else
        {
            var ret = new uint[count];
            int i = 1;
            var path_walker = dad.ending_node;

            while (path_walker != dad.starting_node) // i <= count
            {
                ret[count-i] = path_walker;
                i += 1;
                path_walker = dad.prev_node[path_walker];
            }

            return ret;
        }
    }

    protected void Start()
    {
        // var grid = SetupGraph(new string[]{ "A", "B", "C"}, new uint[]{ 0, 1, 15,
        //                                                                 1, 2, 30 });

        var grid = SetupGrid(5, 5, new uint[] { 0, 0, 0, 0, 0,
                                                0, 1, 1, 1, 1,
                                                0, 1, 0, 0, 0,
                                                0, 1, 1, 1, 0,
                                                0, 0, 0, 0, 0 });
        var algo = StartDijkstra(grid, 12, 10);

        while (!Process(algo)) {}

        var path = GetPath(algo);
        var s = "";

        for (int i = 0; i < path.Length; ++i)
        {
            if (i != 0)
                s += ", ";

            s += "" + path[i];
        }

        Debug.Log(s);
    }
}
