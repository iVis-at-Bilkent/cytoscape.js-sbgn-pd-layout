var Organization = require('./Organization');

function MemberPack(childGraph)
{
    var members = [] /*ArrayList<SbgnPDNode>*/;
    members.concat(childGraph.getNodes());
    
    var org = new Organization();

    this.layout();

    var nodes = [];

    for (var i=0; i<childGraph.getNodes().length; i++)
    {
        nodes[i] = childGraph.getNodes()[i];
    }
}

MemberPack.prototype.layout = function ()
{
    var compar = [] /*new ComparableNode[members.size()]*/;

    var i=0;
    for (var j=0; j<this.members.length; j++)
    {
        compar[i++] = members[j];
    }
    
    compar.sort(this.compareNodes);
    this.members = [];
    
    for (var j=0; j<compar.length; j++)
    {
        this.members.push(compar[j].getNode());
    }
    
    for (var j=0; j<this.members.length; j++)
    {
        this.org.insertNode(members[j]);
    }

    // Compaction c = new Compaction(
    // (ArrayList<SbgnPDNode>) members);
    // c.perform();

};

MemberPack.prototype.getWidth = function ()
{
    return this.org.getWidth();
};

MemberPack.prototype.getHeight = function ()
{
    return this.org.getHeight();
};

MemberPack.prototype.adjustLocations = function (x, y)
{
    this.org.adjustLocations(x, y);
};

MemberPack.prototype.getMembers = function ()
{
    return this.members;
};

MemberPack.prototype.compareNodes = function (nodeA, nodeB)
{
    var aSize = nodeA.getWidth() * nodeA.getHeight();
    var bSize = nodeB.getWidth() * nodeB.getHeight();
    
    if (aSize > bSize)
    {
        return 1;
    }
    else if (aSize < bSize)
    {
        return -1;
    }
    else
    {
        return 0;
    }
    return this.members;
};

module.exports = MemberPack;