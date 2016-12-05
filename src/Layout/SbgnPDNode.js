var CoSENode = require('./CoSENode');
var SbgnPDConstants = require('./SbgnPDConstants');
var PointD = require('./PointD');

// TODO: There is another contructor available in java, do we need it?

function SbgnPDNode(gm, loc, size, vNode, type) 
{
    CoSENode.call(this, gm, loc, size, vNode);
    
    this.type = type;
    this.visited = false;
    
    this.label = vNode ? vNode.label : null;
    this.isDummyCompound = false;
}

SbgnPDNode.prototype = Object.create(CoSENode.prototype);
for (var prop in CoSENode) {
  SbgnPDNode[prop] = CoSENode[prop];
}

SbgnPDNode.prototype.copy = function (/*SbgnPDNode*/ node) 
{
    this.type = node.type;
    this.label = node.label;
    this.setCenter(node.getCenterX(), node.getCenterY());
    this.setChild(node.getChild());
    this.setHeight(node.getHeight());
    this.setLocation(node.getLocation().x, node.getLocation().y);
    this.setNext(node.getNext());
    this.setOwner(node.getOwner());
    this.setPred1(node.getPred1());
    this.setPred2(node.getPred2());
    this.setWidth(node.getWidth());
};

SbgnPDNode.prototype.getSpringForceX = function ()
{
    return this.springForceX;
};

SbgnPDNode.prototype.isComplex = function ()
{
    return this.type.localeCompare(SbgnPDConstants.COMPLEX) === 0;
};

SbgnPDNode.prototype.isInputPort = function ()
{
    return this.type.localeCompare(SbgnPDConstants.INPUT_PORT) === 0;
};

SbgnPDNode.prototype.isOutputPort = function ()
{
    return this.type.localeCompare(SbgnPDConstants.OUTPUT_PORT) === 0;
};

/**
 * This method checks if the given node contains any unmarked complex nodes
 * in its child graph.
*/
SbgnPDNode.prototype.containsUnmarkedComplex = function ()
{
    if (this.getChild() == null)
    {
        return false;
    }
    else
    {
        var numOfChildren = this.getChild().getNodes().length;
        for (var index=0; index<numOfChildren; index++)
        {
            var child = this.getChild().getNodes()[index];
            if (child.isComplex() && !child.visited)
            {
                return true;
            }
        }
        return false;
    }
};

SbgnPDNode.prototype.resetForces = function ()
{
    this.springForceX = 0;
    this.springForceY = 0;
    this.repulsionForceX = 0;
    this.repulsionForceY = 0;
};

SbgnPDNode.prototype.rotateNode = function (/*PointD*/ origin, /*int*/ rotationDegree)
{
    var relativePt = new PointD(
            (this.getCenterX() - origin.x), (this.getCenterY() - origin.y));
    var rotatedPt = new PointD(
            (-Math.signum(rotationDegree) * relativePt.y), 
            (Math.signum(rotationDegree) * relativePt.x));

    this.setCenter(rotatedPt.x + origin.x, rotatedPt.y + origin.y);

    var newHeight = this.getWidth();
    var newWidth = this.getHeight();
    this.setWidth(newWidth);
    this.setHeight(newHeight);
};

/**
 * This method is used for port nodes only
 */
SbgnPDNode.prototype.calcAveragePoint = function ()
{
    var averagePnt = new PointD(0.0, 0.0);
    
    var numOfEdges = this.getEdges().length;
    for (var index=0; index<numOfEdges; index++)
    {
        var edge = this.getEdges()[index];
        
        if (edge.type.localeCompare(SbgnPDConstants.RIGID_EDGE) === 0)
        {
            continue;
        }
        
        averagePnt.x += edge.getOtherEnd(this).getCenterX();
        averagePnt.y += edge.getOtherEnd(this).getCenterY();
    }

    averagePnt.x /= (this.getEdges().length - 1);
    averagePnt.y /= (this.getEdges().length - 1);

    return averagePnt;
};

/**
* This method returns the neighbors of a given node. Notice that the graph
* is directed. Therefore edges should have the given node as the source
* node.
*/
SbgnPDNode.prototype.getChildrenNeighbors = function (/*String*/ edgeType)
{
    var neighbors = [];

    for (var index = 0; index<this.getEdges().length; index++)
    {
        var edge = this.getEdges()[index];

        if ((edge.getSource()== this) && (edge.getTarget() != this))
        {
            var node = edge.getTarget();

            if (edgeType != null && edge.type.localeCompare(edgeType) === 0)
            {
                neighbors.push(node);
            }
            if (edgeType == null)
            {
                neighbors.push(node);
            }
        }
    }
    return neighbors;
};

/**
 * This method updates the bounds of this compound node. If the node is a
 * dummy compound, do not include label and extra margins.
 */
SbgnPDNode.prototype.updateBounds = function ()
{
    // TODO: Do we need handle assertions?
    /*assert this.getChild() != null;*/

    if (this.getChild().getNodes().length !== 0)
    {
        // wrap the children nodes by re-arranging the boundaries
        var childGraph = this.getChild();
        childGraph.updateBounds(true);

        this.rect.x = childGraph.getLeft();
        this.rect.y = childGraph.getTop();

        if ((this.type != null) && 
            (this.type.localeCompare(SbgnPDConstants.DUMMY_COMPOUND) === 0))
        {
            this.setWidth(childGraph.getRight() - childGraph.getLeft());
            this.setHeight(childGraph.getBottom() - childGraph.getTop());
        }
        else
        {
            this.setWidth(childGraph.getRight() - childGraph.getLeft() + 2
                            * SbgnPDConstants.COMPOUND_NODE_MARGIN);
            this.setHeight(childGraph.getBottom() - childGraph.getTop() + 2
                            * SbgnPDConstants.COMPOUND_NODE_MARGIN
                            + SbgnPDConstants.LABEL_HEIGHT);
        }
    }
};
        
module.exports = SbgnPDNode;
