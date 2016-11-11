var CoSELayout = require('./CoSELayout');
var HashMap = require('./HashMap');
var SbgnPDConstants = require('./SbgnPDConstants');
var IGeometry = require('./IGeometry');

SbgnPDLayout.prototype.DefaultCompactionAlgorithmEnum = 
{
    TILING : 0, 
    POLYOMINO_PACKING : 1
};

SbgnPDLayout.prototype.OrientationEnum = 
{
    LEFT_TO_RIGHT : 0, 
    RIGHT_TO_LEFT : 1,
    TOP_TO_BOTTOM : 2, 
    BOTTOM_TO_TOP : 3
};

function SbgnPDLayout() 
{
    CoSELayout.call(this);

    this.rotationRandomizationMethod = 1;

    this.enhancedRatio = 0;
    this.totalEffCount = 0;
    this.compactionMethod = this.DefaultCompactionAlgorithmEnum.TILING;

    this.childGraphMap = new HashMap();          /*Map<SbgnPDNode, LGraph>*/
    this.complexOrder = [];                      /*List<SbgnPDNode>*/
    this.dummyComplexList = [];                  /*List<SbgnPDNode>*/
    this.emptiedDummyComplexMap = new HashMap(); /*Map<SbgnPDNode, LGraph>*/
    this.processNodeList = [];                   /*List<SbgnProcessNode>*/
    
    if (this.compactionMethod === this.DefaultCompactionAlgorithmEnum.TILING)
    {
        this.memberPackMap = new HashMap();      /*Map<SbgnPDNode, MemberPack>*/
    }
};

SbgnPDLayout.prototype = Object.create(CoSELayout.prototype);

for (var prop in CoSELayout) {
  SbgnPDLayout[prop] = CoSELayout[prop];
}

/**
 * @Override This method performs the actual layout on the l-level compound
 *           graph. An update() needs to be called for changes to be
 *           propagated to the v-level compound graph.
 */
SbgnPDLayout.prototype.runSpringEmbedder = function ()
{
    console.log("SBGN-PD Layout is running...");
    this.phaseNumber = 1;
    this.doPhase1();

    this.phaseNumber = 2;
    this.doPhase2();

    // used to calculate - to make sure
    this.recalcProperlyOrientedEdges(true);

    console.log("success ratio: " + this.successRatio);

    this.finalEnhancement();

    console.log("enhanced ratio: " + this.enhancedRatio);

    this.removeDummyCompounds();
};

/**
* At this phase, CoSE is applied for a number of iterations.
*/
SbgnPDLayout.prototype.doPhase1 = function ()
{
    this.maxIterations = SbgnPDConstants.PHASE1_MAX_ITERATION_COUNT;
    this.totalIterations = 0;

    do
    {
        this.totalIterations++;
        if ((this.totalIterations % SbgnPDConstants.CONVERGENCE_CHECK_PERIOD) === 0)
        {
            if (this.isConverged())
            {
                break;
            }

            this.coolingFactor = 
                this.initialCoolingFactor * 
                ((this.maxIterations - this.totalIterations) / this.maxIterations);
        }

        this.totalDisplacement = 0;

        this.graphManager.updateBounds();
        this.calcSpringForces();
        this.calcRepulsionForces();
        this.calcGravitationalForces();
        this.moveNodes();

        this.animate();
    }
    while (this.totalIterations < this.maxIterations);

    this.graphManager.updateBounds();
    this.phase1IterationCount = this.totalIterations;
};

/**
* At this phase, location of single nodes are approximated occasionally.
* Rotational forces are applied. Cooling factor starts from a small value
* to prevent huge changes.
*/
SbgnPDLayout.prototype.doPhase2 = function ()
{
    // dynamic max iteration
    this.maxIterations = 
            Math.log(this.getAllEdges().length + this.getAllNodes().length) * 400;

    // cooling fac is small
    this.initialCoolingFactor = SbgnPDConstants.PHASE2_INITIAL_COOLINGFACTOR;
    this.coolingFactor = this.initialCoolingFactor;

    this.totalIterations = 0;

    do
    {
        this.totalIterations++;

        if ((this.totalIterations % SbgnPDConstants.CONVERGENCE_CHECK_PERIOD) === 0)
        {
                this.successRatio = this.properlyOrientedEdgeCount / this.totalEdgeCountToBeOriented;

                if (this.isConverged() && 
                    successRatio >= SbgnPDConstants.ROTATIONAL_FORCE_CONVERGENCE)
                {
                    break;
                }

                this.coolingFactor = 
                        this.initialCoolingFactor * 
                        ((this.maxIterations - this.totalIterations) / this.maxIterations);
        }

        this.totalDisplacement = 0;

        this.graphManager.updateBounds();

        this.calcSpringForces();
        this.calcRepulsionForces();
        this.calcGravitationalForces();
        this.moveNodes();
        this.animate();
    }
    while (this.totalIterations < this.maxIterations
                    && this.totalIterations < 10000);

    this.phase2IterationCount = this.totalIterations;
    this.graphManager.updateBounds();
};

SbgnPDLayout.prototype.moveNodes = function ()
{
    this.properlyOrientedEdgeCount = 0;
    this.totalEdgeCountToBeOriented = 0;

    // only change single node positions on early stages
    if (this.hasApproximationPeriodReached() && this.coolingFactor > 0.02)
    {
        var numOfProcessNodes = this.processNodeList.length;
        for (var index; index<numOfProcessNodes; index++)
        {
            this.processNodeList[index].applyApproximations();
        }
    }
    
    var numOfProcessNodes = this.processNodeList.length;
    for (var index; index<numOfProcessNodes; index++)
    {
        var sbgnProcessNode = this.processNodeList[index];
        
        // calculate rotational forces for phase 2 only
        if (this.phaseNumber === 2)
        {
            sbgnProcessNode.calcRotationalForces();

            this.properlyOrientedEdgeCount += sbgnProcessNode.properEdgeCount;
            this.totalEdgeCountToBeOriented += 
                    (sbgnProcessNode.consumptionEdges.size() + 
                     sbgnProcessNode.productEdges.size() + 
                     sbgnProcessNode.effectorEdges.size());
            this.successRatio = 
                    this.properlyOrientedEdgeCount / this.totalEdgeCountToBeOriented;
        }
        
        sbgnProcessNode.transferForces();

        sbgnProcessNode.resetForces();
        sbgnProcessNode.inputPort.resetForces();
        sbgnProcessNode.outputPort.resetForces();
    }

    // each time, rotate one process that wants to rotate
    if (((this.totalIterations % SbgnPDConstants.ROTATIONAL_FORCE_ITERATION_COUNT) === 0) && 
        this.phaseNumber === 2)
    {
        rotateAProcess();
    }
            
    // TODO: Original: super.moveNodes();
    CoSELayout.moveNodes();
};

SbgnPDLayout.prototype.hasApproximationPeriodReached = function ()
{
    if((this.totalIterations % 100) === (SbgnPDConstants.APPROXIMATION_PERIOD))
    {
        return true;
    }
    else
    {
        return false;
    }
};

SbgnPDLayout.prototype.rotateAProcess = function ()
{
    var processNodesToBeRotated = [] /*List<SbgnProcessNode>*/;
    
    var numOfProcessNodes = this.processNodeList.length;
    for (var index; index<numOfProcessNodes; index++)
    {
        var sbgnProcessNode = this.processNodeList[index];
        
        if (sbgnProcessNode.isRotationNecessary())
        {
            processNodesToBeRotated.push(sbgnProcessNode);
        }       
    }

    // random selection
    if (processNodesToBeRotated.length > 0)
    {
        var randomIndex = 0;

        if (this.rotationRandomizationMethod === 0)
        {
            randomIndex = this.rouletteWheelSelection(processNodesToBeRotated);

            if (randomIndex === -1)
            {
                console.log("ERROR: no nodes have been selected for rotation");
            }
        }
        else
        {
            randomIndex = (Math.random() * processNodesToBeRotated.lenght);
        }

        var sbgnProcessNode = processNodesToBeRotated[randomIndex];
        sbgnProcessNode.applyRotation();
    }

    // // reset net rotational forces on all processes for next round
    // not used because even if the amount is small, summing up the net
    // force from prev iterations yield better results
    // for (Object o : this.getAllNodes())
    // {
    // if (o instanceof SbgnProcessNode)
    // ((SbgnProcessNode) o).netRotationalForce = 0;
    // }
};

/**
* This method iterates over the process nodes and checks if there exists
* another orientation which maximizes the total number of properly edges.
* If there is, the orientation is changed.
*/
SbgnPDLayout.prototype.finalEnhancement = function ()
{
    var orientationList = [] /*List<Orientation>*/;
    var bestStepResult = 0.0;
    var bestOrientation = null /*Orientation*/;
    var stepAppropriateEdgeCnt = 0.0;
    var totalProperEdges = 0.0;
    var angle = 0.0;

    orientationList.push(this.Orientation.LEFT_TO_RIGHT);
    orientationList.push(this.Orientation.RIGHT_TO_LEFT);
    orientationList.push(this.Orientation.TOP_TO_BOTTOM);
    orientationList.push(this.Orientation.BOTTOM_TO_TOP);
    
    var numOfProcessNodes = this.processNodeList.length;
    for (var i; i<numOfProcessNodes; i++)
    {
        var sbgnProcessNode = this.processNodeList[i];
    
        bestStepResult = sbgnProcessNode.properEdgeCount;
        bestOrientation = null;
        var rememberPropList = []/*List<Boolean>*/;
        var bestPropList = []/*List<Boolean>*/;
        
        var numOfOrientations = orientationList.length;
        for (var j; j<numOfOrientations; j++)
        {
            var orient = orientationList[j];
            
            stepAppropriateEdgeCnt = 0;

            var inputPortTarget = sbgnProcessNode.findPortTargetPoint(true, orient);
            var outputPortTarget = sbgnProcessNode.findPortTargetPoint(false, orient);

            rememberPropList = [];
            
            var numOfConsumptionEdges = sbgnProcessNode.consumptionEdges.length;
            for (var k; k<numOfConsumptionEdges; k++)
            {
                var edge = sbgnProcessNode.consumptionEdges[k];
                var node = edge.getSource();
                angle = IGeometry.calculateAngle(inputPortTarget,
                                sbgnProcessNode.inputPort.getCenter(), 
                                node.getCenter());
                if (angle <= SbgnPDConstants.ANGLE_TOLERANCE)
                {
                    stepAppropriateEdgeCnt++;
                    rememberPropList.push(true);
                }
                else
                {
                    rememberPropList.push(false);
                }
            }
            
            var numOfProductEdges = sbgnProcessNode.productEdges.length;
            for (var k; k<numOfProductEdges; k++)
            {
                var edge = sbgnProcessNode.productEdges[k];
                var node = edge.getTarget();
                angle = IGeometry.calculateAngle(outputPortTarget,
                                sbgnProcessNode.outputPort.getCenter(), 
                                node.getCenter());

                if (angle <= SbgnPDConstants.ANGLE_TOLERANCE)
                {
                    stepAppropriateEdgeCnt++;
                    rememberPropList.push(true);
                }
                else
                {
                    rememberPropList.push(false);
                }         
            }
            
            var numOfEffectorEdges = sbgnProcessNode.effectorEdges.length;
            for (var k; k<numOfEffectorEdges; k++)
            {
                var edge = sbgnProcessNode.effectorEdges[k];
                var node = edge.getSource();
                angle = calcEffectorAngle(orient, sbgnProcessNode.getCenter(), node);

                if (angle <= SbgnPDConstants.EFFECTOR_ANGLE_TOLERANCE)
                {
                    stepAppropriateEdgeCnt++;
                    rememberPropList.push(true);

                }
                else
                {
                    rememberPropList.push(false);
                }
            }

            if (stepAppropriateEdgeCnt > bestStepResult)
            {
                bestStepResult = stepAppropriateEdgeCnt;
                bestOrientation = orient;
                bestPropList = rememberPropList;
            }
        }
        totalProperEdges += bestStepResult;

        // it means a better position has been found
        if (bestStepResult > sbgnProcessNode.properEdgeCount)
        {
            sbgnProcessNode.setOrientation(bestOrientation);
            sbgnProcessNode.properEdgeCount = bestStepResult;

            // mark edges with best known configuration values
            for (var m = 0; m < sbgnProcessNode.consumptionEdges.length; m++)
            {
                sbgnProcessNode.consumptionEdges[i].isProperlyOriented = bestPropList[i];
            }
            for (var n = 0; n < sbgnProcessNode.productEdges.length; n++)
            {
                sbgnProcessNode.productEdges[i].isProperlyOriented = bestPropList[i + sbgnProcessNode.consumptionEdges.length];
            }
            for (var o = 0; o < sbgnProcessNode.effectorEdges.size(); o++)
            {
                    sbgnProcessNode.effectorEdges[i].isProperlyOriented = 
                            bestPropList[i + 
                                (sbgnProcessNode.consumptionEdges.length + 
                                    sbgnProcessNode.productEdges.lenght)];
            }
        }
    }

    this.properlyOrientedEdgeCount = totalProperEdges;
    this.enhancedRatio = totalProperEdges / totalEdgeCountToBeOriented;

    var numOfProcessNodes = this.processNodeList.length;
    for (var i; i<numOfProcessNodes; i++)
    {
        totalEffCount += this.processNodeList[i].effectorEdges.length;
    }
};

module.exports = SbgnPDLayout;
