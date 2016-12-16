var CoSEEdge = require('./CoSEEdge');
var SbgnPDConstants = require('./SbgnPDConstants');

function SbgnPDEdge(source, target, vEdge, type) 
{
    CoSEEdge.call(this, source, target, vEdge);
    
    this.type = type;                // String (from LGraphObject)
    this.correspondingAngle = 0;     // int
    this.isProperlyOriented = false; // boolean
}

SbgnPDEdge.prototype = Object.create(CoSEEdge.prototype);
for (var prop in CoSEEdge) {
  SbgnPDEdge[prop] = CoSEEdge[prop];
}

SbgnPDEdge.prototype.copy = function (/*SbgnPDEdge*/ edge) 
{
    // TODO: Do we really have access to these two functions?
    this.setSource(edge.getSource());
    this.setTarget(edge.getTarget());
    
    this.label = edge.label;
    this.type = edge.type;
    this.correspondingAngle = edge.correspondingAngle;
    this.isProperlyOriented = edge.isProperlyOriented;
    this.idealLength = edge.idealLength;
    this.isInterGraph = edge.isInterGraph;
    this.bendpoints = edge.bendpoints;
    this.isOverlapingSourceAndTarget = edge.isOverlapingSourceAndTarget;
    this.lca = edge.lca;
    this.length = edge.length;
    this.lengthX = edge.lengthX;
    this.lengthY = edge.lengthY;
    this.sourceInLca = edge.sourceInLca;
};

SbgnPDEdge.prototype.isEffector = function () 
{
    if(this.type.localeCompare(SbgnPDConstants.MODULATION) === 0 || 
       this.type.localeCompare(SbgnPDConstants.STIMULATION) === 0 || 
       this.type.localeCompare(SbgnPDConstants.CATALYSIS) === 0 || 
       this.type.localeCompare(SbgnPDConstants.INHIBITION) === 0 || 
       this.type.localeCompare(SbgnPDConstants.NECESSARY_STIMULATION) === 0)
   {
       return true;
   }
   else
   {
       return false;
   }
};

SbgnPDEdge.prototype.isRigidEdge = function () 
{
    if(this.type.localeCompare(SbgnPDConstants.RIGID_EDGE) === 0 )
    {
        return true;
    }
    else
    {
        return false;
    }
};

module.exports = SbgnPDEdge;
