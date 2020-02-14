#include<SFML/Graphics.hpp>
#include<SFML/Audio.hpp>
#include<SFML/Window.hpp>
#include<iostream>
#include<functional>
#include<math.h>    
#include<float.h>
#include<vector>
#include<set>
#include<string.h>
#include<string>
#include<sstream>
using namespace std;
using namespace sf;
#define num 60      //number of cells in a row

//--------Dijkstra--------
vector<pair<int,int> > pathD;        //Shortest pathD
bool sptSet[num][num];      //explored cells
void findmin(float dist[num][num],int& min_x,int& min_y){
    float mini=FLT_MAX;
    for(int i=0;i<num;i++)
        for(int j=0;j<num;j++)
            if(sptSet[i][j]==false && dist[i][j]<mini){
                mini=dist[i][j];
                min_x=i;
                min_y=j;
            }
}
void findpath(pair<int,int> previous[num][num],float dist[num][num],int dest_x,int dest_y,int source_x,int source_y){
    cout<<"\nLength of Dijkstra path is: "<<dist[dest_x][dest_y]<<endl;
    while(previous[dest_x][dest_y].first!=source_x || previous[dest_x][dest_y].second!=source_y){        // both simultaneously equal to source coordinates
        sf::sleep(milliseconds(10));        //delay shortest pathD
        cout << "Visiting x = " << previous[dest_x][dest_y].first << "  " << "and y = " << previous[dest_x][dest_y].second << endl;
        pathD.push_back(make_pair(previous[dest_x][dest_y].first,previous[dest_x][dest_y].second));
        int save_x=dest_x,save_y=dest_y;
        dest_x=previous[save_x][save_y].first;
        dest_y=previous[save_x][save_y].second;
    }
}
void dijkstra(int source_x,int source_y,int dest_x,int dest_y,int grid[num][num]){
    pair<int,int> previous[num][num];
    float dist[num][num];
    for(int i=0;i<num;i++)
        for(int j=0;j<num;j++)
            dist[i][j]=FLT_MAX;
    dist[source_x][source_y]=0.0;
    int found=0;
    for(int i=0;i<num && found==0;i++) {
        for(int j=0;j<num && found==0;j++){
            int min_x=0,min_y=0;
            findmin(dist,min_x,min_y);
            sptSet[min_x][min_y]=true;
            if(sptSet[dest_x][dest_y]==true){
                found=1;
                break;
            }
            sf::sleep(milliseconds(1));        //delay exploration
            int possibleX[] = {0, 0, 1, -1, 1, -1, -1, 1};
            int possibleY[] = {1, -1, 0, 0, 1, 1, -1, -1};

            for(int i = 0; i < 8; ++i) {
                int newRow = min_x + possibleX[i];
                int newCol = min_y + possibleY[i];
                if(grid[newRow][newCol]==1 && sptSet[newRow][newCol]==false && dist[newRow][newCol]>dist[min_x][min_y]+1.0){
                dist[newRow][newCol]=dist[min_x][min_y]+1.0;
                previous[newRow][newCol]=make_pair(min_x,min_y);
                }
            }
        }
    }
        
    findpath(previous,dist,dest_x,dest_y,source_x,source_y);
}
//--------Astar--------
typedef pair<int,int> Pair;
typedef pair<float,pair<int,int>> Ppair;
bool closedList[num][num];
vector<Pair> pathA;
struct cell{
    int parent_x,parent_y;
    float f,g,h;
    cell() : f(FLT_MAX),g(FLT_MAX),h(FLT_MAX),parent_x(-1),parent_y(-1) {};
};
bool isDestination(int row,int col,Pair dest){
    if(row==dest.first && col==dest.second)
        return true;
    else
        return false;
}
float calculateHvalue(int row,int col,Pair dest){
    int dx=abs(dest.first-row);
    int dy=abs(dest.second-col);
    return abs(dx-dy)+sqrt(2)*min(dx,dy);       //Diagonal D=1,D2=sqrt(2)
}
void tracePath(Pair source,Pair dest,cell cellDetails[][num]){
    int i=cellDetails[dest.first][dest.second].parent_x,j=cellDetails[dest.first][dest.second].parent_y;
    while(!(i==source.first && j==source.second)){
        sf::sleep(milliseconds(10));        //delay shortest path
        cout << "Visiting x = " << i << "  " << "and y = " << j << endl;
        pathA.push_back(make_pair(i,j));
        //pathSum=pathSum+cellDetails[i][j].g;
        int temp_i=i;
        int temp_j=j;
        i=cellDetails[temp_i][temp_j].parent_x;     //Solved substitution bug
        j=cellDetails[temp_i][temp_j].parent_y;
    }
    cout<<"\nLength of A* path(g) is: "<<cellDetails[dest.first][dest.second].g<<endl;
}
void Astar(Pair source,Pair dest,int grid[][num]){
    set<Ppair> openList;
    cell cellDetails[num][num];
    int i= source.first,j=source.second;
    cellDetails[i][j].f=0.0;
    cellDetails[i][j].g=0.0;
    cellDetails[i][j].h=0.0;
    cellDetails[i][j].parent_x=i;
    cellDetails[i][j].parent_y=j;
    openList.insert(make_pair(0.0,make_pair(i,j)));
    bool destFound=false;
    int possibleX[] = {0, 0, 1, -1, 1, -1, -1, 1};
    int possibleY[] = {1, -1, 0, 0, 1, 1, -1, -1};
    while(!openList.empty()){
        Ppair p=*openList.begin();
        openList.erase(openList.begin());
        int i=p.second.first,j=p.second.second;
        closedList[i][j]=true;
        sf::sleep(milliseconds(1));        //delay exploration
        if(isDestination(i,j,dest)==true){
            cout<<"Destination Found\n";
            destFound=true;
            break;      //out of while loop
        }
        for(int k = 0; k < 8; ++k) {
            int newRow = i + possibleX[k];
            int newCol = j + possibleY[k];
            if(grid[newRow][newCol]==1 && closedList[newRow][newCol]==false){
            cell successor;
            successor.g=cellDetails[i][j].g+1.0;
            successor.h=calculateHvalue(newRow,newCol,dest);
            successor.f=successor.g+successor.h;
            successor.parent_x=i;
            successor.parent_y=j;
            if(cellDetails[newRow][newCol].g>successor.g){        //Not in openList or bigger 'g' in openList
                cellDetails[newRow][newCol]=successor;
                openList.insert(make_pair(successor.f,make_pair(newRow,newCol)));
            }
            }
        }
    }
    if(destFound==false)
        cout<<"Destination cell not found.\n";
    else
        tracePath(source,dest,cellDetails);
}
//--------main()--------
int main(){
    int filled[num][num];       //whether cell is colored
    int grid[60][60];       //map with obstacle
    for(int i=0;i<60;i++)
        for(int j=0;j<60;j++){
            if(i==0||i==59||j==0||j==59)        //walls
                grid[i][j]=0;
            else
                grid[i][j]=1;
        }
     for(int i=0;i<num;i++)
        for(int j=0;j<num;j++){
            sptSet[i][j]=false;     //dijkstra all unexplored
            filled[i][j]=0;     //all uncolored
        }
    memset(closedList,false,sizeof(closedList));        //Astar all unexplored
    int source_x=2,source_y=2,dest_x=50,dest_y=56;      //Origin(2,3)->Goal(50,56)
    Thread threadD(std::bind(&dijkstra,source_x,source_y,dest_x,dest_y,grid));
    Thread threadA(std::bind(&Astar,make_pair(source_x,source_y),make_pair(dest_x,dest_y),grid));
    RenderWindow window(VideoMode(800,600),"Grid");
    // // Text
    sf::Font font;
    font.loadFromFile("arial.ttf");
    sf::Text text1("DIJKSTRA",font,15);
    sf::Text text2("A*",font,24);

    // Shapes
    RectangleShape buttonStartD(Vector2f(75,25));       //button dijkstra
    buttonStartD.setFillColor(Color::Green);
    RectangleShape buttonStartA(Vector2f(75,25));       //button Astar
    buttonStartA.setFillColor(Color::Magenta);
   
    RectangleShape rectangle(Vector2f(10,10));      //default box :White
    rectangle.setFillColor(Color::White);
    RectangleShape brectangle(Vector2f(10,10));     //Black box
    brectangle.setFillColor(Color::Black);
    RectangleShape grectangle(Vector2f(10,10));     //Green
    grectangle.setFillColor(Color::Green);
    grectangle.setOutlineThickness(2);
    grectangle.setOutlineColor(Color::Red);
    RectangleShape mrectangle(Vector2f(10,10));     //Magenta
    mrectangle.setFillColor(Color::Magenta);
    mrectangle.setOutlineThickness(2);
    mrectangle.setOutlineColor(Color::Red);
    RectangleShape blueRectangle(Vector2f(10,10));
    blueRectangle.setFillColor(Color::Blue);
    blueRectangle.setOutlineThickness(2);
    blueRectangle.setOutlineColor(Color::Black);
    RectangleShape rrectangle(Vector2f(10,10));
    rrectangle.setFillColor(Color::Red);
    rrectangle.setOutlineThickness(2);
    rrectangle.setOutlineColor(Color::Red);
    RectangleShape yrectangle(Vector2f(10,10));
    yrectangle.setFillColor(Color::Yellow);
    // Display
    while(window.isOpen()){
        Event event;
        while(window.pollEvent(event)){
            if(event.type==Event::Closed)
                window.close();
            if(event.type==Event::KeyPressed && event.key.code==Keyboard::Space)
                window.close();
            if(event.type==Event::MouseButtonPressed && event.mouseButton.button==Mouse::Left){
                int X=event.mouseButton.x;
                int Y=event.mouseButton.y;
                int row=Y/10;       //Reversed notion of row & column
                int col=X/10;
                if(grid[row][col]==0&&row<60&&col<60)
                    grid[row][col]=1;
                else if(row<60&&col<60)
                    grid[row][col]=0;
                if(row<60&col<60)cout<<"Cell "<<row<<" , "<<col<<" state is: "<<grid[row][col]<<endl;
                if(X>600&&X<675&&Y>0&&Y<25){
                    threadD.launch();
                }
                if(X>600&&X<675&&Y>75&&Y<100){
                    threadA.launch();
                }
            }
        }
        window.clear();
        buttonStartD.setPosition(600,0);
        window.draw(buttonStartD);      //Dijkstra launch
        buttonStartA.setPosition(600,75);
        window.draw(buttonStartA);      //Astar launch
        text1.setPosition(600,0);       //Dijkstra text
        text2.setPosition(630,75);
        window.draw(text1);
        window.draw(text2);
        stringstream ss1,ss2;
        ss1<<pathD.size();       //int2string
        ss2<<pathA.size();
        
        if(!pathA.empty()){
            for(int i=0;i<int(pathA.size());i++){
                mrectangle.setPosition(pathA[i].second*10,pathA[i].first*10);     //Reversed notion of row & column
                window.draw(mrectangle);        //final pathA
                filled[pathA[i].first][pathA[i].second]=1;
            }
        }
        if(!pathD.empty()){
            for(int i=0;i<int(pathD.size());i++){
                grectangle.setPosition(pathD[i].second*10,pathD[i].first*10);     //Reversed notion of row & column
                window.draw(grectangle);        //final pathD
                filled[pathD[i].first][pathD[i].second]=1;
            }
        }
        blueRectangle.setPosition(source_y*10,source_x*10);
        window.draw(blueRectangle);     //source
        filled[source_x][source_y]=1;
        rrectangle.setPosition(dest_y*10,dest_x*10);
        window.draw(rrectangle);        //destination
        filled[dest_x][dest_y]=1;
        for(int i=0;i<=590 ;i+=10)
            for(int j=0;j<=590;j+=10){
                if(grid[i/10][j/10]==0){
                    brectangle.setOutlineThickness(2);
                    brectangle.setOutlineColor(Color::Red);
                    brectangle.setPosition(j,i);
                    window.draw(brectangle);        //User's obstacle
                }
                if(sptSet[i/10][j/10]==true && filled[i/10][j/10]==0){
                    yrectangle.setOutlineThickness(2);
                    yrectangle.setOutlineColor(Color::Red);
                    yrectangle.setPosition(j,i);
                    window.draw(yrectangle);        // Explored Cells by dijkstra
                }
                if(closedList[i/10][j/10]==true && filled[i/10][j/10]==0){
                    yrectangle.setOutlineThickness(2);
                    yrectangle.setOutlineColor(Color::Red);
                    yrectangle.setPosition(j,i);
                    window.draw(yrectangle);        // Explored  Cells by A*
                }
                if(grid[i/10][j/10]==1 && sptSet[i/10][j/10]==false && closedList[i/10][j/10]==false && filled[i/10][j/10]==0){     //not in dijkstra & A*
                    rectangle.setOutlineThickness(2);
                    rectangle.setOutlineColor(Color::Red);
                    rectangle.setPosition(j,i);
                    window.draw(rectangle);     //default white cells
                }
            }
        window.display();
    }
    return 0;
}

