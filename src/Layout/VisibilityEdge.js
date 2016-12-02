var CoSEEdge = require('./CoSEEdge');

function VisibilityEdge(source, target, vEdge) {
  CoSEEdge.call(this, source, target, vEdge);
}

VisibilityEdge.prototype = Object.create(CoSEEdge.prototype);
for (var prop in CoSEEdge) {
  VisibilityEdge[prop] = CoSEEdge[prop];
}

/**
* We want the length of a visibility edge calculated as the distance
* between borders of two nodes, not the distance between center points.
* Edges have to be vertical or horizontal.
*/
// @Override
VisibilityEdge.prototype.updateLength = function ()
{
    if (this.source.getBottom() <= this.target.getTop())
    {
        this.lengthX = 0;
        this.lengthY = this.source.getBottom() - this.target.getTop();
    } 
    else if (this.source.getRight() <= this.target.getLeft())
    {
        this.lengthX = this.source.getRight() - this.target.getLeft();
        this.lengthY = 0;
    }
    // else
    // System.out.println("unexpected edge");

    this.length = Math.sqrt((this.lengthX * this.lengthX) + (this.lengthY * this.lengthY));
};

module.exports = VisibilityEdge;
