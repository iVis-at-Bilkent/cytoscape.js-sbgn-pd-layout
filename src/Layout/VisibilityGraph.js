var Integer = require('./Integer');
var RectangleD = require('./RectangleD');

var LGraph = require('./LGraph');
var VisibilityEdge = require('./VisibilityEdge');
var Compaction = require('./Compaction');

function VisibilityGraph(parent, graphMgr, vGraph) 
{
    LGraph.call(this, parent, graphMgr, vGraph);
    
    this.direction = null;
}

VisibilityGraph.prototype = Object.create(LGraph.prototype);
for (var prop in LGraph) {
  VisibilityGraph[prop] = LGraph[prop];
}

/**
* Create a new visibility graph. Compare each vertices in the graph to see
* if they are visible to each other. If two vertices can see each other and
* they see each other in the desired direction, add an edge between them.
*/
VisibilityGraph.prototype.construct = function (d, vertices)
{
    this.init(vertices);

    var nodes = this.getNodes();
    this.direction = d;

    // check the visibility between each two vertex
    for (var i = 0; i < nodes.length; i++)
    {
        for (var j = i + 1; j < nodes.length; j++)
        {
            var node1 = nodes[i];
            var node2 = nodes[j];

            var result = this.findVisibilityDirection(node1, node2);

            // if they are visible, create edge.
            if (result !== 0)
            {
                this.createEdge(node1, node2);
            }
        }
    }
};

/**
* This method adds the given nodes to the graph.
*/
VisibilityGraph.prototype.init = function (vertices)
{
    // create the new graph with given vertices
    var numOfVertices = vertices.length;
    for (var i=0; i<numOfVertices; i++)
    {
        this.add(vertices[i]);
    }
};

/**
* Given two nodes, check their visibility. Two nodes are visible to each
* other if there exists an infinite ray that intersects them without
* intersecting any other nodes in between those two.
* 
* @return 1: vertical, 2: horizontal (if any edge found). Otherwise, return
*         0
*/
VisibilityGraph.prototype.findVisibilityDirection = function (p, q)
{
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        // ensure that p points to the leftmost element
        if (q.getLeft() < p.getLeft() && 
            p.getLeft() < (q.getLeft() + q.getWidth()))
        {
            var temp = p;
            p = q;
            q = temp;
        }

        // check if there exists a ray
        if (p.getLeft() <= q.getLeft() && 
            q.getLeft() <= (p.getLeft() + p.getWidth()))
        {
            if (this.sweepIntersectedArea(p, q))
            {
                return 1;
            }
        }
    }

    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        // ensure that p points to the upper element
        if (q.getTop() < p.getTop() && 
            p.getTop() < (q.getTop() + q.getHeight()))
        {
            var temp = p;
            p = q;
            q = temp;
        }

        // check if there exists a ray
        if (p.getTop() <= q.getTop() && 
            q.getTop() <= (p.getTop() + p.getHeight()))
        {
            if (this.sweepIntersectedArea(p, q))
            {
                return 2;
            }
        }
    }
    return 0;
};

/**
* Starting from the intersection area between p and q, walk on a line
* perpendicular to the desired direction. If there is an edge that does not
* intersect with any other nodes, this is a valid edge.
* 
* @return true if an edge exists. false otherwise.
*/
VisibilityGraph.prototype.sweepIntersectedArea = function (p, q)
{
    var edge;
    var isValid;
    var start = 0;
    var end = 0;
    var result;

    // find the sweep line borders
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        start = q.getLeft();
        end = Math.min(p.getRight(), q.getRight());
    }
    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        start = q.getTop();
        end = Math.min(p.getBottom(), q.getBottom());
    }

    // if they intersect only on the borders, immediately return false.
    if (start === end)
    {
        return false;
    }

    // check for all intersected area
    for (var sweepPoint = start; sweepPoint <= end; sweepPoint++)
    {
        isValid = true;
        edge = this.tryConstructingEdge(p, q, sweepPoint);

        // if an edge is constructed, check its validity
        if (edge !== null)
        {
            result = this.checkIntermediateNodes(p, q, edge, sweepPoint);

            if (sweepPoint === result)
                isValid = true;
            else
            {
                sweepPoint = result;
                isValid = false;
            }
        }
        if (isValid)
            return true;
    }

    return false;
};

/**
* This method tries to construct an edge(RectangleD shape) between two
* nodes. The parameter i indicates the starting point of the edge. For
* example, if a vertical edge to be constructed, i is the top coordinate.
* For horizontal, i is the leftmost coordinate.
* 
* @return edge. if no edge can be constructed, returns null.
*/
VisibilityGraph.prototype.tryConstructingEdge = function (p, q, i)
{
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        // create an edge from upper to lower or return false:does not
        // exist
        if (p.getTop() < q.getTop() && 
            p.getBottom() <= q.getTop())
        {
            return new RectangleD(i, p.getBottom(), 1, (q.getTop() - p.getBottom()));
        }
        else if (q.getTop() < p.getTop() && 
                 (q.getTop() + q.getHeight()) <= p.getTop())
        {
            return new RectangleD(i, q.getBottom(), 1, (p.getTop() - q.getBottom()));
        }
        else
        {
            return null;
        }
    }
    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        // create an edge from leftmost to right or return false:does
        // not exist
        if (p.getLeft() < q.getLeft() && 
            p.getRight() <= q.getLeft())
        {
            return new RectangleD(p.getRight(), i, (q.getLeft() - p.getRight()), 1);
        }
        else if (q.getLeft() < p.getLeft() && 
                 q.getRight() <= p.getLeft())
        {
            return new RectangleD(q.getRight(), i, (p.getLeft() - q.getRight()), 1);
        }
        else
        {
            return null;
        }
    }

    return null;
};

/**
* This method checks if the given edge intersects any nodes except the
* source and target nodes. If an intersection is found, update the sweep
* point to the end of the intersected node. Otherwise, do not change the
* point.
*/
VisibilityGraph.prototype.checkIntermediateNodes = function (p, q, edge, sweepPoint)
{
    for (var j = 0; j < this.getNodes().length; j++)
    {
        var intermediateNode = this.getNodes()[j];

        if (intermediateNode !== p && intermediateNode !== q)
        {
            // if there is an intersection, edge is not valid
            if (edge.intersects(intermediateNode.getRect()))
            {
                // jump to the end of intersected node
                if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
                    sweepPoint = (intermediateNode.getRight() + 1);
                else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
                    sweepPoint = (intermediateNode.getBottom() + 1);

                break;
            }
        }
    }

    return sweepPoint;
};

/**
* This class creates an edge between the given nodes using the given
* direction. While adding the edge, be careful about the source and target
* i.e. if we want to get a vertical visibility graph, (A -> B) A should
* have a lower y coordinate (upper). Similarly, for horizontal visibility
* graph (A -> B): A is on the left, has lower x coordinate.
*/
VisibilityGraph.prototype.createEdge = function (node1, node2)
{
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        if (node1.getTop() < node2.getTop())
            this.add(new VisibilityEdge(node1, node2, null), node1, node2);
        else
            this.add(new VisibilityEdge(node2, node1, null), node2, node1);
    }

    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        if (node1.getLeft() < node2.getLeft())
            this.add(new VisibilityEdge(node1, node2, null), node1, node2);
        else
            this.add(new VisibilityEdge(node2, node1, null), node2, node1);
    }

    // calculate newly added edge's length.
    this.getEdges()[this.getEdges().length - 1].updateLength();
};

/**
* For each edge having s as its target node, find and return the shortest
* one. Returns null if could not find an edge.
*/
VisibilityGraph.prototype.findShortestEdge = function (s)
{
    var shortestEdge = null;
    var minLength = Integer.MAX_VALUE;

    for (var i = 0; i < this.getEdges().length; i++)
    {
        var e = this.getEdges()[i];

        e.updateLength();
        if (e.getTarget() === s && e.getLength() < minLength)
        {
            shortestEdge = e;
            minLength = e.getLength();
        }
    }

    return shortestEdge;
};

module.exports = VisibilityGraph;
