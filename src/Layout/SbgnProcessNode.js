var IGeometry = require('./IGeometry');
var PointD = require('./PointD');

var SbgnPDNode = require('./SbgnPDNode');
var SbgnPDConstants = require('./SbgnPDConstants');

SbgnProcessNode.prototype.OrientationEnum = 
{
    BOTTOM_TO_TOP : 0, 
    TOP_TO_BOTTOM : 1,
    LEFT_TO_RIGHT : 2,
    RIGHT_TO_LEFT : 3
};

SbgnProcessNode.prototype.RotationPriorityEnum = 
{
    NINETY_DEGREE : 0, 
    SWAP : 1,
    NO_ROTATION : 2
};

function SbgnProcessNode(gm, loc, size, vNode, type) 
{
    SbgnPDNode.call(this, gm, loc, size, vNode, type);

    this.netRotationalForce = 0;
    this.consumptionEdges = [];
    this.productEdges = [];
    this.effectorEdges = [];
    
    this.parentCompound = null;
    this.inputPort = null;
    this.outputPort = null;
    
    this.orientation = null;
    this.rotationPriority = null;
    
    this.idealEdgeLength = 0.0;
    this.netRotationalForce = 0.0;
    this.properEdgeCount = 0.0;
}

SbgnProcessNode.prototype = Object.create(SbgnPDNode.prototype);
for (var prop in SbgnPDNode) {
  SbgnProcessNode[prop] = SbgnPDNode[prop];
}

/**
* Connect the port node to its process node (parent) and connect the edges
* of neighbor nodes to the port node by considering their types (for both
* input port and output port)
*/
SbgnProcessNode.prototype.reconnectEdges = function (idealEdgeLength)
{
    this.idealEdgeLength = idealEdgeLength;
    
    // change connections from process node&neighbors to port&neighbors.
    for (var i = 0; i < this.getEdges().length; i++)
    {
        var sEdge = this.getEdges()[i];

        if (sEdge.type === SbgnPDConstants.CONSUMPTION)
        {
            this.getEdges().splice(this.getEdges().indexOf(sEdge), 1);

            sEdge.setTarget(this.inputPort);
            this.inputPort.getEdges().push(sEdge);
            i--;
        }
        else if (sEdge.type === SbgnPDConstants.PRODUCTION)
        {
            this.getEdges().splice(this.getEdges().indexOf(sEdge), 1);

            sEdge.setSource(this.outputPort);
            this.outputPort.getEdges().push(sEdge);
            i--;
        }
        else if (sEdge.isEffector())
        {
            this.getEdges().splice(this.getEdges().indexOf(sEdge), 1);

            sEdge.setTarget(this.parentCompound);
            this.parentCompound.getEdges().push(sEdge);
            i--;
        }
    }
};

SbgnProcessNode.prototype.connectNodes = function (parentCompound, inputPort, outputPort)
{
    this.parentCompound = parentCompound;
    this.parentCompound.isDummyCompound = true;
    this.inputPort = inputPort;
    this.outputPort = outputPort;
    this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;

    // initial placement. place input to the left of the process node,
    // output to the right
    outputPort.setCenter(this.getCenterX() + SbgnPDConstants.RIGID_EDGE_LENGTH, 
                         this.getCenterY());
    inputPort.setCenter(this.getCenterX() - SbgnPDConstants.RIGID_EDGE_LENGTH, 
                        this.getCenterY());
};

/**
* Check if the process is eligible for rotation. First check if a
* 180-degree is possible (as it is more critical).
*/
SbgnProcessNode.prototype.isRotationNecessary = function ()
{
    // normalize the amount (per iteration)
    this.netRotationalForce /= (SbgnPDConstants.ROTATIONAL_FORCE_ITERATION_COUNT * 
                                 (this.consumptionEdges.length + 
                                  this.productEdges.length + 
                                  this.effectorEdges.length));

    if (this.isSwapAvailable())
    {
         this.rotationPriority = this.RotationPriorityEnum.SWAP;
         return true;
    }
    else if (Math.abs(this.netRotationalForce) > SbgnPDConstants.ROTATION_90_DEGREE)
    {
         this.rotationPriority = this.RotationPriorityEnum.NINETY_DEGREE;
         return true;
    }
    else
    {
         this.rotationPriority = this.RotationPriorityEnum.NO_ROTATION;
         return false;
    }
};

/**
* If the percentage of obtuse angles exceeds the threshold, swap is
* required.
*/
SbgnProcessNode.prototype.isSwapAvailable = function ()
{
    var obtuseAngleCnt = 0.0;
    var acuteAngleCnt = 0.0;

    var numOfConsumptionEdges = this.consumptionEdges.length;
    for (var i=0; i<numOfConsumptionEdges; i++)
    {
        var edge = this.consumptionEdges[i];
        
        if (Math.abs(edge.correspondingAngle) > 90)
            obtuseAngleCnt++;
        else
            acuteAngleCnt++;
    }
    
    var numOfProductEdges = this.productEdges.length;
    for (var i=0; i<numOfProductEdges; i++)
    {
        var edge = this.productEdges[i];
        
        if (Math.abs(edge.correspondingAngle) > 90)
                obtuseAngleCnt++;
        else
                acuteAngleCnt++;
    }

    if (obtuseAngleCnt / (obtuseAngleCnt + acuteAngleCnt) > SbgnPDConstants.ROTATION_180_DEGREE)
        return true;
    else
        return false;
};

/**
* This method rotates the associated compound (and its children: process
* and ports).
*/
SbgnProcessNode.prototype.applyRotation = function ()
{
    if (this.rotationPriority === this.RotationPriorityEnum.NINETY_DEGREE)
    {
        if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM)
        {
            if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation = this.OrientationEnum.RIGHT_TO_LEFT;
            }
            else if (netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;
            }
        }
        else if (this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
        {
            if (this.netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;
            }
            else if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation = this.OrientationEnum.RIGHT_TO_LEFT;
            }
        }
        else if (this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
        {
            if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation = this.OrientationEnum.BOTTOM_TO_TOP;
            }
            else if (this.netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation =  this.OrientationEnum.TOP_TO_BOTTOM;
            }
        }
        else if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT)
        {
            if (this.netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation = this.OrientationEnum.BOTTOM_TO_TOP;
            }
            else if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation =  this.OrientationEnum.TOP_TO_BOTTOM;
            }
        }
    }

    else if (this.rotationPriority === this.RotationPriorityEnum.SWAP)
    {
        var tempCenter = this.inputPort.getCenter();
        
        this.inputPort.setCenter(this.outputPort.getCenterX(), 
                            this.outputPort.getCenterY());
        this.outputPort.setCenter(tempCenter.x, tempCenter.y);

        if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM)
            this.orientation = this.OrientationEnum.BOTTOM_TO_TOP;
        else if (this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
            this.orientation = this.OrientationEnum.TOP_TO_BOTTOM;
        else if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT)
            this.orientation = this.OrientationEnum.RIGHT_TO_LEFT;
        else if (this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
            this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;
    }

    this.netRotationalForce = 0;
};


/**
* Given a compound node, this method recursively rotates the compound node
* and its members.
*/
SbgnProcessNode.prototype.rotateCompound = function (rotationDegree)
{
    this.rotateNode(this.getCenter(), rotationDegree);
    this.inputPort.rotateNode(this.getCenter(), rotationDegree);
    this.outputPort.rotateNode(this.getCenter(), rotationDegree);
    this.parentCompound.updateBounds();
};

SbgnProcessNode.prototype.calcRotationalForces = function ()
{
    this.netRotationalForce += this.calcProperlyOrientedEdges();
};

/**
* This method calculates all angles between process and its edges (prod,
* cons, eff) and marks them as properly oriented or not. Returned value is
* the amount of desire to rotate at this step. The returned value should be
* then manually added to netRotationalForce (if aim is to calculate
* netrotationalforce)
*/
SbgnProcessNode.prototype.calcProperlyOrientedEdges = function ()
{
    var inputRotSum = 0;
    var outputRotSum = 0;
    var effectorRotSum = 0;
    var stepSum = 0;
    var result;
    this.properEdgeCount = 0;

    // if the neighbors of port nodes have not been detected yet, find them.
    if (this.consumptionEdges.length === 0 && this.productEdges.length === 0)
    {
        this.initLists();
    }

    // find ideal positions
    var inputPortTarget = this.findPortTargetPoint(true, this.orientation);
    var outputPortTarget = this.findPortTargetPoint(false, this.orientation);

    for (var nodeIndex = 0; nodeIndex < this.consumptionEdges.length; nodeIndex++)
    {
        result = this.calcRotationalForce(true, nodeIndex, inputPortTarget);
        if (Math.abs(result) <= SbgnPDConstants.ANGLE_TOLERANCE)
        {
            this.properEdgeCount++;
        }
        inputRotSum += result;
    }
    for (var nodeIndex = 0; nodeIndex < this.productEdges.length; nodeIndex++)
    {
        result = this.calcRotationalForce(false, nodeIndex, outputPortTarget);
        if (Math.abs(result) <= SbgnPDConstants.ANGLE_TOLERANCE)
        {
            this.properEdgeCount++;
        }
        outputRotSum += result;
    }

    for (var nodeIndex = 0; nodeIndex < this.effectorEdges.length; nodeIndex++)
    {
        result = this.calcEffectorAngle(nodeIndex);
        if (Math.abs(result) <= SbgnPDConstants.EFFECTOR_ANGLE_TOLERANCE)
                this.properEdgeCount++;

        effectorRotSum += Math.abs(result);
    }

    // add total effector rotational force with the same sign of
    // step sum because it does not matter for an effector node
    // either rotate to left or right. therefore support the rotation
    // direction of each iteration.
    stepSum = inputRotSum - outputRotSum;
    stepSum = stepSum + (Math.sign(stepSum) * Math.abs(effectorRotSum));

    return stepSum;
};

/**
* This method returns the signed angle between a node and its corresponding
* port and the target point.
*/
SbgnProcessNode.prototype.calcRotationalForce = function (isInputPort, nodeIndex, targetPoint)
{
    var node;
    var centerPoint;

    if (isInputPort)
    {
        node = this.consumptionEdges[nodeIndex].getSource();
        centerPoint = this.inputPort.getCenter();
    }
    else
    {
        node = this.productEdges[nodeIndex].getTarget();
        centerPoint = this.outputPort.getCenter();
    }

    var angle = IGeometry.calculateAngle(targetPoint, centerPoint, node.getCenter());

    if (isInputPort)
        angle *= this.isLeft(targetPoint, centerPoint, node.getCenter(), SbgnPDConstants.INPUT_PORT);
    else
        angle *= this.isLeft(targetPoint, centerPoint, node.getCenter(), SbgnPDConstants.OUTPUT_PORT);

    this.saveInformation(isInputPort, nodeIndex, angle);

    return angle;
};

/**
* Calculates the angle between an effector edge and its process node. An
* effector edge has process (in this case the dummy compound) as its target
* node and the effector itself as the source.
*/
SbgnProcessNode.prototype.calcEffectorAngle = function (nodeIndex)
{
    var eff = this.effectorEdges[nodeIndex].getSource();
    var targetPnt = new PointD();
    var centerPnt = this.getCenter();

    // find target point
    if (this.isHorizontal())
    {
        targetPnt.x = this.getCenterX();

        if (eff.getCenterY() > this.getCenterY())
            targetPnt.y = this.getCenterY() + this.idealEdgeLength;
        else
            targetPnt.y = this.getCenterY() - this.idealEdgeLength;
    }
    else if (this.isVertical())
    {
        targetPnt.y = this.getCenterY();

        if (eff.getCenterX() > this.getCenterX())
                targetPnt.x = this.getCenterX() + this.idealEdgeLength;
        else
                targetPnt.x = this.getCenterX() - this.idealEdgeLength;
    }

    var angle = IGeometry.calculateAngle(targetPnt, centerPnt, eff.getCenter());

    this.effectorEdges[nodeIndex].correspondingAngle = angle;

    if (Math.abs(angle) <= SbgnPDConstants.EFFECTOR_ANGLE_TOLERANCE)
        this.effectorEdges[nodeIndex].isProperlyOriented = true;
    else
        this.effectorEdges[nodeIndex].isProperlyOriented = false;

    return angle;
};

SbgnProcessNode.prototype.applyApproximations = function ()
{
    // if there is only one single-edge consumption, move it to ideal
    // otherwise move towards multiedge node

    if (this.consumptionEdges.length === 1 && 
        this.consumptionEdges[0].getSource().getEdges().length === 1)
    {
        this.approximateForSingleNodes(this.inputPort, this.consumptionEdges[0].getSource());
    }
    else
    {
        this.approximateForMultipleNodes(this.inputPort);
    }

    if (this.productEdges.length === 1 && 
        this.productEdges[0].getTarget().getEdges().length === 1)
    {    
        this.approximateForSingleNodes(this.outputPort, this.productEdges[0].getTarget());
    }
    else
    {
        approximateForMultipleNodes(this.outputPort);
    }

    this.approximateEffectors();
};

SbgnProcessNode.prototype.approximateForSingleNodes = function (port, node)
{
    var targetPt = new PointD();
    var newPoint = new PointD();
    
    if (port.isInputPort())
    {
        targetPt = this.findPortTargetPoint(true, this.orientation);
    }    
    
    else if (port.isOutputPort())
    {
        targetPt = this.findPortTargetPoint(false, this.orientation);   
    }
    
    newPoint.x = targetPt.x
                    + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                    - SbgnPDConstants.APPROXIMATION_DISTANCE;
    newPoint.y = targetPt.y
                    + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                    - SbgnPDConstants.APPROXIMATION_DISTANCE;

    node.setCenter(newPoint.x, newPoint.y);
};

/**
* Given the port node, this method finds all consumption(or production)
* nodes of this port node. The idea is to move each single-edge
* consumptions(products) of a process closer to neighbor multi-edge nodes
* to keep them close to each other.
*/
SbgnProcessNode.prototype.approximateForMultipleNodes = function (port)
{
    var oneEdgeNodes = [];
    var multiEdgeNodes = [];
    var nodeOfInterest = null;
    var targetPt = new PointD();

    // get all non-rigid edges of port node
    var numOfEdges = port.getEdges().length;
    for (var i=0; i<numOfEdges; i++)
    {
        var edge = port.getEdges()[i];

        if (edge.isRigidEdge())
        {
            continue;
        }

        // node of interest depends on the direction of the edge
        if (port.isInputPort())
        {
            nodeOfInterest = edge.getSource();   
        }        
        else if (port.isOutputPort())
        {
            nodeOfInterest = edge.getTarget();
        }

        if (nodeOfInterest.getEdges().length === 1) // single node    
        {
            oneEdgeNodes.push(nodeOfInterest);
        }
        else if (nodeOfInterest.getEdges().length > 1) // multiedge node
        {
            multiEdgeNodes.push(nodeOfInterest);
        }
    }

    if (port.isInputPort())
    {
        targetPt = this.findPortTargetPoint(true, this.orientation);   
    }        
    else if (port.isOutputPort())
    {
        targetPt = this.findPortTargetPoint(false, this.orientation);
    }        
    
    // move
    if (oneEdgeNodes.length > 0)
    {
        moveOneEdgeNodes(oneEdgeNodes, multiEdgeNodes, targetPt);
    }    
};

/**
* Single-edge nodes are moved around the center point of a multi-edge node.
* If all the neighbor nodes of that port node are single-edged, then one of
* them is chosen randomly and the others are placed around it.
* 
* @param targetPt
*/
SbgnProcessNode.prototype.moveOneEdgeNodes = function (oneEdgeNodes, multiEdgeNodes, targetPt)
{
    var approximationPnt = new PointD(0, 0);
    var randomIndex = -1;
    var approximationNode = null;
    var newPoint = new PointD();

    // if there are more than one multi edge nodes, select the highly
    // connected one
    if (multiEdgeNodes.length > 0)
    {
        approximationNode = multiEdgeNodes[0];
        
        var numOfMultiEdgeNodes = multiEdgeNodes.length;
        for (var i=0; i<numOfMultiEdgeNodes; i++)
        {
            var node = multiEdgeNodes[i];

            if (node.getEdges().length > approximationNode.getEdges().length)
            {
                approximationNode = node;
            }
        }
    }

    // if there are no multi edge nodes, randomly select one
    else if (multiEdgeNodes.length === 0)
    {
        randomIndex = (Math.random() * oneEdgeNodes.length);
        approximationNode = oneEdgeNodes[randomIndex];
    }

    approximationPnt = approximationNode.getCenter();

    // move single nodes around the approximation point
    var numOfOneEdgeNodes = oneEdgeNodes.length;
    for (var i=0; i<numOfOneEdgeNodes; i++)
    {
        var s = oneEdgeNodes[i];
        
        // if they belong to different graphs, do not move
        // if (approximationNode.getOwner() != s.getOwner())
        // continue;
        newPoint.x = approximationPnt.x
                        + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                        - SbgnPDConstants.APPROXIMATION_DISTANCE;
        newPoint.y = approximationPnt.y
                        + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                        - SbgnPDConstants.APPROXIMATION_DISTANCE;

        s.setCenter(newPoint.x, newPoint.y);
    }
};

module.exports = SbgnProcessNode;
