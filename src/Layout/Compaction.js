var VisibilityEdge = require('./VisibilityEdge');
var VisibilityGraph = require('./VisibilityGraph');
var SbgnPDConstants = require('./SbgnPDConstants');

function Compaction(vertices) 
{
    this.orderedNodeList = [] /*ArrayList<SbgnPDNode>()*/;
    this.vertices = vertices;
    
    this.visGraph = null;
    this.direction = null;
}

Compaction.prototype.CompactionDirectionEnum = 
{
    VERTICAL : 0, 
    HORIZONTAL : 1
};

/**
* Two times do the following: (first for vertical, second horizontal) First
* create a visibility graph for the given elements. The visibility graph is
* always a DAG, so perform a topological sort on the elements (the node
* that has in-degree 0 comes first), perform compaction.
*/
Compaction.prototype.perform = function ()
{
    this.algorithmBody(this.CompactionDirectionEnum.VERTICAL);
    this.removeVisibilityEdges();

    this.algorithmBody(this.CompactionDirectionEnum.HORIZONTAL);
    this.removeVisibilityEdges();

};

Compaction.prototype.removeVisibilityEdges = function ()
{
    var numOfVertices = this.vertices.length;
    for(var i=0; i<numOfVertices; i++)
    {
        var sbgnNode = this.vertices[i];

        for(var j = 0; j < sbgnNode.getEdges().length; j++)
        {
            var objEdge = sbgnNode.getEdges()[j];

            if( objEdge instanceof VisibilityEdge)
            {
                sbgnNode.getEdges().splice(i, 1);
                i--;
            }
        }
    }	
};

Compaction.prototype.algorithmBody = function (direction)
{
    this.direction = direction;
    this.visGraph = new VisibilityGraph(null, null, null);

    // construct a visibility graph given the direction and vertices
    this.visGraph.construct(this.direction, vertices);

    if (this.visGraph.getEdges().length > 0)
    {
        this.topologicallySort();
        this.compactElements();
    }

    // positions of the vertices has changed. Update them.
    this.vertices = this.visGraph.getNodes();
};

/**
* Perform a DFS on the given graph nodes and then output the nodes in
* reverse order.
*/
Compaction.prototype.topologicallySort = function ()
{
    // ensure that the vertices have not been marked as visited.
    for (var i = 0; i < this.visGraph.getNodes().length; i++)
    {
        var s = this.visGraph.getNodes()[i];
        s.visited = false;
    }

    // ensure that the list is empty
    this.orderedNodeList = [];
    this.DFS();
    this.orderedNodeList = this.reverseList(this.orderedNodeList);
};

Compaction.prototype.DFS = function ()
{
    for (var i = 0; i < this.visGraph.getNodes().length; i++)
    {
        var s = this.visGraph.getNodes()[i];
        if (!s.visited)
        {
            this.DFS_Visit(s);
        }
    }
};

Compaction.prototype.DFS_Visit = function (s)
{
    var neighbors = s.getChildrenNeighbors(null);

    if (neighbors.length === 0)
    {
        s.visited = true;
        this.orderedNodeList.push(s);
        return;
    }

    var numOfNeighbours = neighbors.length;
    for (var i=0; i<numOfNeighbours; i++)
    {
        if (!neighbors[i].visited)
        {
            this.DFS_Visit(neighbors[i]);
        }
    }

    s.visited = true;
    this.orderedNodeList.push(s);
};

/**
* Reverse the element order of a given list
*/
Compaction.prototype.reverseList = function (originalList)
{
    var reverseOutput = [];
    for (var i = originalList.length - 1; i >= 0; i--)
    {
        reverseOutput.push(originalList[i]);
    }

    return reverseOutput;
};

/**
* This method visits the list that is the result of topological sort. For
* each node in that list, it looks for its incoming edges and finds the
* shortest one. Translates the node wrt the shortest edge.
*/
Compaction.prototype.compactElements = function ()
{
    var distance = 0.0;
    
    var orderedNodeListLength = this.orderedNodeList.length;
    for (var i=0; i<orderedNodeListLength; i++)
    {
        var sbgnPDNode = this.orderedNodeList[i];
        
        // find shortest incoming edge
        var edge = this.visGraph.findShortestEdge(sbgnPDNode);

        if (edge != null)
        {
            distance = edge.getLength();

            if (this.direction === this.CompactionDirectionEnum.VERTICAL)
            {
                // bring the node closer to the source node and respect the
                // buffer.
                if (distance > SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER)
                {
                    sbgnPDNode.setLocation(
                            sbgnPDNode.getLeft(),
                            (sbgnPDNode.getTop() - 
                                    (distance - SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER)));
                }
                else
                {
                    sbgnPDNode.setLocation(
                            sbgnPDNode.getLeft(), 
                            edge.getOtherEnd(sbgnPDNode).getBottom()
                                    + SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER);
                }
            }
            else if (this.direction === this.CompactionDirectionEnum.HORIZONTAL)
            {
                if (distance > SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER)
                {
                    sbgnPDNode.setLocation(
                            (sbgnPDNode.getLeft() - (distance - SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER)),
                            sbgnPDNode.getTop());
                }
                else
                {
                    sbgnPDNode.setLocation(
                            edge.getOtherEnd(sbgnPDNode).getRight()
                                            + SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER,
                            sbgnPDNode.getTop());
                }
            }
        }
    }
};

module.exports = Compaction;


