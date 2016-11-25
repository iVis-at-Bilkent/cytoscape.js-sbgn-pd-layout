function Polyomino() 
{
    // the number of cells
    this.l = 0;

    //the resulting placement coordinates
    this.x = 0;
    this.y = 0;

    this.label = "";

    // polyomino cells
    this.coord = [];
}

module.exports = Polyomino;