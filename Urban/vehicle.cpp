#include "traffic_light.h"
#include "vehicle.h"
#include "sensor.h"
#include "measures.h"

#include <qdebug.h>

bool switch_matriz;
bool switch_vehicles;

int n_hor_streets; //An integer that represents the amount of horizontal streets
int m_ver_streets; //An integer that represents the amount of vertical streets
int d_hor_street; //horizontal distance of the streets
int d_ver_street; //vertical distance of the streets

int d_side_block; //distance of the smallest area that is surrounded by streets
int n_blocks; //An integer that represents the amount of blocks

//Pointers for read
SCell **pr_horizontal_streets;
SCell **pr_vertical_streets;
SCell **pr_intersections;

//Pointers for write
SCell **pw_horizontal_streets;
SCell **pw_vertical_streets;
SCell **pw_intersections;

SCell **horizontal_streets_A;//Two-dimensional array representing the streets in horizontal direction
SCell **vertical_streets_A;//Two-dimensional array representing the streets in vertical direction
SCell **intersections_A;//Two dimensional array that represents the intersections between horizontal and vertical streets

SCell **horizontal_streets_Z;//Two-dimensional array representing the streets in horizontal direction
SCell **vertical_streets_Z;//Two-dimensional array representing the streets in vertical direction
SCell **intersections_Z;//Two dimensional array that represents the intersections between horizontal and vertical streets

int size_vehicles;

SVehicle *pr_vehicles;
SVehicle *pw_vehicles;

SVehicle *vehicles_A;
SVehicle *vehicles_Z;

vector<SVehicle> random_pos;
vector<SVehicle> random_pos_h;
vector<SVehicle> random_pos_v;

float density;
float density_h;
float density_v;

float percentage_autonomous;

int total_cell;
int n_cell_h;
int n_cell_v;
float p_turn;

int collisions;

float save_velocity;
float save_flux;

int n_exp;
int n_ticks;
float size_step;

int error_fp = 0;
int error_fn = 0;

int test_error_fp = 0;
int test_error_fn = 0;

FILE *fp_f;
FILE *fp_v;

FILE *fp_fopt;
FILE *fp_vopt;


bool behavior_green = true;
int vmax = 6; //velocidad maxima
int M = 1;// 2 Capacidad de frenado (5.00m / Dx = 2.5m)

float *Ra; //Probabilidad de acelerar
float Rs = 0.01; //probabilidad de sobre frenar
float R0 = 0.8;
float Rd = 1.0;

int ls = 1;//Longitud de los vehiculos
int vs = 2; //(5 / 5) + 1; //Velocidad baja
float delta_x = 5;
float delta_v = 1.0; //cambie la formula para ajustar a una celda de la longitud del vehiculo

unsigned int **acc;
unsigned int **keep;
unsigned int **dec0;
unsigned int *dbreak;

//int metodo;
int inicio;

// random generator function:
int myrandom (int i)
{
    return rand() % i;
}

double frand()
{
    return (double)rand() / RAND_MAX;
}

int allocateMemoryCity()
{
    int i, m, n;

    //Pointer of read an write to Matrices A and Z
    pr_intersections = new SCell*[n_hor_streets];
    pr_horizontal_streets = new SCell*[n_hor_streets];
    pr_vertical_streets = new SCell*[m_ver_streets];

    pw_intersections = new SCell*[n_hor_streets];
    pw_horizontal_streets = new SCell*[n_hor_streets];
    pw_vertical_streets = new SCell*[m_ver_streets];

    //Matrices A
    intersections_A = new SCell*[n_hor_streets];
    for (i = 0; i < n_hor_streets; i++)
        intersections_A[i] = new SCell[m_ver_streets];

    //reserved by n horizontal streets
    horizontal_streets_A = new SCell*[n_hor_streets];
    for (i = 0; i < n_hor_streets; i++)
        horizontal_streets_A[i] = new SCell[d_hor_street];

    //reserved by vertical distance streets
    vertical_streets_A = new SCell*[m_ver_streets];
    for (i = 0; i < m_ver_streets; i++)
        vertical_streets_A[i] = new SCell[d_ver_street];

    //Matrices Z
    intersections_Z = new SCell*[n_hor_streets];
    for (i = 0; i < n_hor_streets; i++)
        intersections_Z[i] = new SCell[m_ver_streets];

    //reserved by n horizontal streets
    horizontal_streets_Z = new SCell*[n_hor_streets];
    for (i = 0; i < n_hor_streets; i++)
        horizontal_streets_Z[i] = new SCell[d_hor_street];

    //reserved by vertical distance streets
    vertical_streets_Z = new SCell*[m_ver_streets];
    for (i = 0; i < m_ver_streets; i++)
        vertical_streets_Z[i] = new SCell[d_ver_street];

    return 0;
}

void FreeCity()
{
   int i;

    delete [] vehicles_Z;
    delete [] vehicles_A;

    for (i = 0; i < m_ver_streets; i++)
        delete [] vertical_streets_Z[i];
    delete [] vertical_streets_Z;

    for (i = 0; i < n_hor_streets; i++)
        delete [] horizontal_streets_Z[i];
    delete [] horizontal_streets_Z;

    //Matrices Z
    for (i = 0; i < n_hor_streets; i++)
        delete [] intersections_Z[i];
    delete [] intersections_Z;

    for (i = 0; i < m_ver_streets; i++)
        delete [] vertical_streets_A[i];
    delete [] vertical_streets_A;

    for (i = 0; i < n_hor_streets; i++)
        delete [] horizontal_streets_A[i];
    delete [] horizontal_streets_A;

    //Matrices A
    for (i = 0; i < n_hor_streets; i++)
        delete [] intersections_A[i];
    delete [] intersections_A;

    delete [] pw_intersections;
    delete [] pw_horizontal_streets;
    delete [] pw_vertical_streets;

    delete [] pr_intersections;
    delete [] pr_horizontal_streets;
    delete [] pr_vertical_streets;

    for (i = 0; i < vmax + delta_v; i++)
       delete [] acc[i];
    delete [] acc;

    for (i = 0; i < vmax + delta_v; i++)
        delete [] keep[i];
    delete [] keep;

    for (i = 0; i < vmax + 1; i++)
        delete [] dec0[i];
    delete [] dec0;

    delete [] dbreak;

    delete [] Ra;

}

//The city is set to agree to the number of horizontal and vertical streets, and the distance of the blocks
int CityBuilding(int n_hor, int m_ver, int d_blk)
{
    int i, n, m;

    //Sets values ​​valid in the numbers of horizontal and vertical streets, and the distance of the blocks.
    if (n_hor < 1)
        n_hor_streets = 1;
    else
        n_hor_streets = n_hor;

    if (m_ver < 1)
        m_ver_streets = 1;
    else
        m_ver_streets = m_ver;

    if (d_blk < 2)
        d_side_block = 2;
    else
        d_side_block = d_blk;

    //Las celdas solo consideran las calles (no todo el terreno)
    total_cell = 2 * d_side_block * n_hor_streets * m_ver_streets + n_hor_streets * m_ver_streets;
    n_cell_h = n_hor_streets * (d_side_block * m_ver_streets) + m_ver_streets;
    n_cell_v = m_ver_streets * (d_side_block * n_hor_streets) + n_hor_streets;

    //The required memory is reserved dynamically.
    //The horizontal distance is greater when more vertical streets.
    //The vertical distance is greater when there is more horizontal streets.
    d_hor_street = d_side_block * m_ver_streets + m_ver_streets;
    d_ver_street = d_side_block * n_hor_streets + n_hor_streets;

    allocateMemoryCity();

    //The streets are divided into cells. The state of the cells are classified as follows:
    //0 - The cell is empty
    //1 - The cell contains a vehicle
    //2 - The cell is an intersection (for what needs to redirected to the array of intersections).

    SwitchMatricesRW();
    resetCityWrite();
    SwitchMatricesRW();
    resetCityWrite();

    //printCity();

    return 0;
}

void SetPositionVehicle(char type_street, int y, int x, int id)
{

    if (id < 0 || id >= size_vehicles)
        id = 0;

    if (type_street == 'H'){

        if (y < 0 || y >= n_hor_streets)
            y = 0;

        if (x < 0 || x >= d_hor_street)
            x = 0;

        pw_vehicles[id].position.x = x;
        pw_vehicles[id].position.y = y;
    }
    else {

        if (y < 0 || y >= m_ver_streets)
            y = 0;

        if (x < 0 || x >= d_ver_street)
            x = 0;

        pw_vehicles[id].position.x = x;
        pw_vehicles[id].position.y = y;
    }
}

SPosition GetPositionVehicle(int id)
{
   SPosition pos;

   if (id < 0 || id >= size_vehicles)
       id = 0;

   pos = pr_vehicles[id].position;

   return pos;
}

void SetVelocityVehicle(int spd, int id)
{
    if (id < 0 || id >= size_vehicles)
        id = 0;

     pw_vehicles[id].speed = spd;
}

int GetVelocityVehicle(int id)
{
    int speed;

    if (id < 0 || id >= size_vehicles)
        id = 0;

    speed = pr_vehicles[id].speed;

    return speed;
}

void SetTypeStreetVehicle(char type, int id)
{
    if (id < 0 || id >= size_vehicles)
        id = 0;

    pw_vehicles[id].type_street = type;
}

void SetDirectionVehicle(char dir, int id)
{
    if (id < 0 || id >= size_vehicles)
        id = 0;

     pw_vehicles[id].direction = dir;
}

char GetDirectionVehicle(int id)
{
    char direction;

    if (id < 0 || id >= size_vehicles)
        id = 0;

    direction = pr_vehicles[id].direction;

    return direction;
}

void SetColorVehicle(struct Color color, int id)
{
    if (id < 0 || id >= size_vehicles)
        id = 0;

     pw_vehicles[id].color = color;
}

struct Color GetColorVehicle(int id)
{
    struct Color col;

    if (id < 0 || id >= size_vehicles)
        id = 0;

    col = pr_vehicles[id].color;

    return col;
}



char GetTypeStreetVehicle(int id)
{
    char type_street;

    if (id < 0 || id >= size_vehicles)
        id = 0;

    type_street = pr_vehicles[id].type_street;

    return type_street;
}


void SetVisibleVehicle(int id, bool visible)
{
    if (id < 0 || id >= size_vehicles)
        id = 0;

    pw_vehicles[id].visible = visible;

}

bool GetVisibleVehicle(int id)
{
    bool visible;

    if (id < 0 || id >= size_vehicles)
        id = 0;

    visible = pr_vehicles[id].visible;

    return visible;
}

bool GetAutonomousVehicle(int id)
{
   bool autonomous;

   if (id < 0 || id >= size_vehicles)
       id = 0;

   autonomous = pr_vehicles[id].autonomous;

   return autonomous;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void SetValueCellStreet(char type_street, int y, int x, int value, bool visible, int id)
{
    if (type_street == 'H'){

        if (y < 0 || y >= n_hor_streets)
            y = 0;

        if (x < 0 || x >= d_hor_street)
            x = 0;

        if (pw_horizontal_streets[y][x].value == 2) {

            int m = x / (1 + d_side_block);

            if (pw_intersections[y][m].value == 1){
                collisions++;
                //if (PRINT_COL == 1)
                //qDebug() << "No. Choques: " << collisions;
             //   printf ("No. Choques: %d", collisions);
            }
            //else {

            pw_intersections[y][m].value = value;
            pw_intersections[y][m].id = id;
            pw_intersections[y][m].visible = visible;
            //}
        }
        else {
            pw_horizontal_streets[y][x].value = value;
            pw_horizontal_streets[y][x].id = id;

            pw_horizontal_streets[y][x].visible = visible;
        }

    }
    else if (type_street == 'V'){

        if (y < 0 || y >= m_ver_streets)
            y = 0;

        if (x < 0 || x >= d_ver_street)
            x = 0;

        if (pw_vertical_streets[y][x].value == 2){

            int n = x / (1 + d_side_block);

            if (pw_intersections[n][y].value == 1){
                collisions++;
                //if (PRINT_COL == 1)
                //  qDebug() << "No. Choques: " << collisions;
                //printf ("No. Choques: %d", collisions);
            }

            pw_intersections[n][y].value = value;
            pw_intersections[n][y].id = id;
            pw_intersections[n][y].visible = visible;

            //pw_intersections[n][y].value = 0;
            //pw_intersections[n][y].id = id;
        }
        else{
            pw_vertical_streets[y][x].value = value;
            pw_vertical_streets[y][x].id = id;

            pw_vertical_streets[y][x].visible = visible;
        }
    }

}

bool isIntersection(char type_street, int y, int x)
{
    if (type_street == 'H'){
        if (pr_horizontal_streets[y][x].value == 2)
            return true;
    }
    else {
        if (pr_vertical_streets[y][x].value == 2)
           return true;
    }

    return false;
}


int GetValueCellStreet(char type_street, int y, int x)
{
    int value;

    value = 0;
    if (type_street == 'H'){

        if (y < 0 || y >= n_hor_streets) {
            printf ("Error en H, y = ", y);
            return 0;
        }

        if (x < 0 || x >= d_hor_street) {
            printf ("Error en H, x = ", x);
            return 0;
        }

        if (pr_horizontal_streets[y][x].value == 2) {

            int m = x / (1 + d_side_block);
            value = pr_intersections[y][m].value;
        }
        else
            value = pr_horizontal_streets[y][x].value;

    }
    else if (type_street == 'V'){

        if (y < 0 || y >= m_ver_streets) {
            printf ("Error en V, y = ", y);
            return 0;
        }

        if (x < 0 || x >= d_ver_street) {
            printf ("Error en V, x = ", x);
            return 0;
        }

        if (pr_vertical_streets[y][x].value == 2) {
            int n = x / (1 + d_side_block);
            value = pr_intersections[n][y].value;
        }
        else
            value = pr_vertical_streets[y][x].value;
    }

    return value;
}

void SetIDCellStreet(char type_street, int y, int x, int id)
{

    if (type_street == 'H'){
        if (y < 0 || y >= n_hor_streets) {
            printf ("Error en H, y = ", y);
            return;
        }

        if (x < 0 || x >= d_hor_street) {
            printf ("Error en H, x = ", x);
            return;
        }

        if (pr_horizontal_streets[y][x].value == 2) {
            int m = x / (1 + d_side_block);
            pw_intersections[y][m].id = id;
        }
        else
            pw_horizontal_streets[y][x].id = id;
    }
    else if (type_street == 'V'){
        if (y < 0 || y >= m_ver_streets) {
            printf ("Error en V, y = ", y);
            return;
        }

        if (x < 0 || x >= d_ver_street) {
            printf ("Error en V, x = ", x);
            return;
        }

        if (pr_vertical_streets[y][x].value == 2) {
            int n = x / (1 + d_side_block);
            pw_intersections[n][y].id = id;
        }
        else
            pw_vertical_streets[y][x].id = id;
    }

}

int GetIDCellStreet(char type_street, int y, int x)
{
    int id;

    id = -1;
    if (type_street == 'H'){
        if (y < 0 || y >= n_hor_streets) {
            printf ("Error en H, y = ", y);
            return 0;
        }

        if (x < 0 || x >= d_hor_street) {
            printf ("Error en H, x = ", x);
            return 0;
        }

        if (pr_horizontal_streets[y][x].value == 2) {
            int m = x / (1 + d_side_block);
            id = pr_intersections[y][m].id;
        }
        else
            id = pr_horizontal_streets[y][x].id;
    }
    else if (type_street == 'V'){
        if (y < 0 || y >= m_ver_streets) {
            printf ("Error en V, y = ", y);
            return 0;
        }

        if (x < 0 || x >= d_ver_street) {
            printf ("Error en V, x = ", x);
            return 0;
        }

        if (pr_vertical_streets[y][x].value == 2) {
            int n = x / (1 + d_side_block);
            id = pr_intersections[n][y].id;
        }
        else
            id = pr_vertical_streets[y][x].id;
    }

    return id;
}


bool GetVisibleCellStreet(char type_street, int y, int x)
{
    bool visible;

    if (type_street == 'H'){
        if (y < 0 || y >= n_hor_streets) {
            printf ("Error en H, y = ", y);
            return 0;
        }

        if (x < 0 || x >= d_hor_street) {
            printf ("Error en H, x = ", x);
            return 0;
        }

        if (pr_horizontal_streets[y][x].value == 2) {
            int m = x / (1 + d_side_block);
            visible = pr_intersections[y][m].visible;
        }
        else
            visible = pr_horizontal_streets[y][x].visible;
    }
    else if (type_street == 'V'){
        if (y < 0 || y >= m_ver_streets) {
            printf ("Error en V, y = ", y);
            return 0;
        }

        if (x < 0 || x >= d_ver_street) {
            printf ("Error en V, x = ", x);
            return 0;
        }

        if (pr_vertical_streets[y][x].value == 2) {
            int n = x / (1 + d_side_block);
            visible = pr_intersections[n][y].visible;
        }
        else
            visible = pr_vertical_streets[y][x].visible;
    }

    return visible;
}

int Rule_184(char type_street, int y, int x, char direction)
{

    int front;
    int speed;

    if (type_street == 'H') {
        if (y < 0 || y >= n_hor_streets)
            y = 0;
        if (x < 0 || x >= d_hor_street)
            x= 0;

        if (direction == 'R'){
            if (x == d_hor_street - 1)
                front = 0;
            else
                front = x + 1;
        }
        else {
            if (x == 0)
                front = d_hor_street - 1;
            else
                front = x - 1;
        }
    }
    else if (type_street == 'V') {
        if (y < 0 || y >= m_ver_streets)
            y = 0;
        if (x < 0 || x >= d_ver_street)
            x= 0;

        if (direction == 'R'){
            if (x == d_ver_street - 1)
                front = 0;
            else
                front = x + 1;
        }
        else {
            if (x == 0)
                front = d_ver_street - 1;
            else
                front = x - 1;
        }
    }

    if (GetValueCellStreet(type_street, y, front) == 1)
        speed = 0;
    else
        speed = 1;

    return speed;
}

int Rule_252(char type_street, int y, int x, char direction)
{
    int speed;

    speed = 0;

    return speed;
}


void InializedCity(int n_h_streets, int m_v_streets, int d_s_block, float dens_h, float dens_v, float p_t, float per_auto)
{
    //PARAMETROS //////////////////////

    switch_matriz = true;

    total_cell = 0;
    velocity = 0;
    flux = 0;
    velocity_total = 0;
    flux_total = 0;
    n_ticks = 2700;


    n_hor_streets = n_h_streets; //An integer that represents the amount of horizontal streets
    m_ver_streets = m_v_streets; //An integer that represents the amount of vertical streets
    d_side_block = d_s_block; //distance of the smallest area that is surrounded by streets

    p_turn = p_t; //probabilidad de girar en las intersecciones

    CityBuilding(n_hor_streets, m_ver_streets, d_side_block);
    DistribuiteVehicles(dens_h, dens_v, per_auto);
    CalcularTablas();//calcular las distancias fuera de linea
    CalcularRa();//calcular probabilidad Ra fuera de linea

}

void CalcularRa()
{
    int i;

    Ra = new float[vmax + 1];

    for (i = 0; i < vmax + 1; i++)
        Ra[i] = min(Rd, R0 + (float)i * ((Rd - R0) / (float)vs));

   /* printf ("Ra: ");
    for (i = 0; i < vmax + 1; i++)
        printf ("%f ", Ra[i]);*/

}

int CalcularTablas()
{

    int i;
    int dnacc, dnkeep, dndec, dnbreak;
    int dp = 0;

    dnacc = dnkeep = dndec = dnbreak = 0;

    acc = new unsigned int*[vmax + 1];
    for (i = 0; i < vmax + delta_v; i++)
        acc[i] = new unsigned int[vmax + 1];

    keep = new unsigned int*[vmax + 1];
    for (i = 0; i < vmax + delta_v; i++)
        keep[i] = new unsigned int[vmax + 1];

    dec0 = new unsigned int*[vmax + 1];
    for (i = 0; i < vmax + 1; i++)
        dec0[i] = new unsigned int[vmax + 1];

    dbreak = new unsigned int[vmax + 1];

    int vn = 0, vp = 0;

    for (vn = 0; vn <= vmax; vn++) {

        for (i = 0; i <= (vn + delta_v) / M; i++)
            dnacc += (vn + delta_v) - (i * M);

        for (i = 0; i <= (vn) / M; i++)
            dnkeep += (vn - i * M);

        for (i = 0; i <= ((vn - delta_v) / M); i++)
            dndec += (vn - delta_v) - (i * M);

        for (i = 0; i <= ((vn - M) / M); i++)
            dnbreak += (vn - M) - (i * M);

        dndec = (dndec < 0) ? 0 : dndec;
        dnkeep = (dnkeep < 0) ? 0 : dnkeep;
        dnacc = (dnacc < 0) ? 0 : dnacc;
        dnbreak = (dnbreak < 0) ? 0 : dnbreak;

        for (vp = 0; vp <= vmax; vp++) {

            for (i = 0; i <= (vp - M) / M; i++)
                dp += (vp - M) - (i * M);

            dp = (dp < 0) ? 0 : dp;

            acc[vn][vp] = ((dnacc - dp) < 0) ? 0 : dnacc - dp;
            keep[vn][vp] = ((dnkeep - dp) < 0) ? 0 : dnkeep - dp;
            dec0[vn][vp] = ((dndec - dp) < 0) ? 0 : dndec - dp;

            dp = 0;
        }

        dbreak[vn] = (dnbreak < 0) ? 0 : dnbreak;


        dnacc = dnkeep = dndec = dnbreak = 0;
    }

    return 0;

}

int RunSimulation(int tick)
{

    if (metodo_light == 1)
        RunSimulationGreenWave(tick);
    else
        RunSimulationSelfOrganization(tick);

}

int DistribuiteVehicles(float d_h, float d_v, float per_auto)
{

    int i;
    int m, n;

    if (d_h <= 1.0)
       density_h = d_h;
    else
       density_h = 0.0;

    if (d_v <= 1.0)
       density_v = d_v;
    else
       density_v = 0.0;

    density = (density_h + density_v) / 2.0;

    int size_vehicles_h = (density_h * n_cell_h) >= 1 ? density_h * n_cell_h : 1;
    int size_vehicles_v = (density_v * n_cell_v) >= 1 ? density_v * n_cell_v : 1;

    size_vehicles = size_vehicles_h + size_vehicles_v;

    percentage_autonomous = per_auto;

    if (percentage_autonomous < 0.0)
        percentage_autonomous = 0.0;
    else if (percentage_autonomous > 1.0)
         percentage_autonomous = 1.0;


    // qDebug() << "Numero h y s:" << h_size_vehices << v_size_vehices;
    // qDebug() << "Numero h y s:" << random_pos_hor.size() << random_pos_ver.size();

    pr_vehicles = NULL;
    pw_vehicles = NULL;

    vehicles_A = new SVehicle[size_vehicles];
    vehicles_Z = new SVehicle[size_vehicles];

    SwitchVehiclesRW();

    random_pos_h.clear();
    random_pos_v.clear();
    random_pos.clear();

    int id = 0;

    for (n = 0; n < n_hor_streets; n++) {
        for (i = 0; i < d_hor_street; i++) {

            SVehicle tmp_vehicle;

            tmp_vehicle.id = id;

            tmp_vehicle.type_street = 'H';
            tmp_vehicle.position.y = n;
            tmp_vehicle.position.x = i;
            tmp_vehicle.speed = 0;
            tmp_vehicle.visible = true;
            tmp_vehicle.autonomous = false;
            tmp_vehicle.color.r = 0;
            tmp_vehicle.color.g = 0,
            tmp_vehicle.color.b = 255;


            if (n % 2 == 0)
                tmp_vehicle.direction = 'R';
            else
                tmp_vehicle.direction = 'L';

            random_pos_h.push_back(tmp_vehicle);

            id++;
        }
    }


    for (m = 0; m < m_ver_streets; m++) {
        for (i = 0; i < d_ver_street; i++) {

            if (GetValueCellStreet('V', m, i) != 2) { //No incluir otra vez interecciones

                SVehicle tmp_vehicle;

                tmp_vehicle.id = id;
                tmp_vehicle.type_street = 'V';
                tmp_vehicle.position.y = m;
                tmp_vehicle.position.x = i;
                tmp_vehicle.speed = 0;
                tmp_vehicle.visible = true;
                tmp_vehicle.autonomous = false;
                tmp_vehicle.color.r = 0;
                tmp_vehicle.color.g = 0,
                tmp_vehicle.color.b = 255;

                if (m % 2 == 0)
                    tmp_vehicle.direction = 'R';
                else
                    tmp_vehicle.direction = 'L';

                random_pos_v.push_back(tmp_vehicle);

                id++;
            }
        }
    }

    random_shuffle(random_pos_h.begin(), random_pos_h.end(), myrandom);
    random_pos_h.resize(size_vehicles_h);

    random_shuffle(random_pos_v.begin(), random_pos_v.end(), myrandom);
    random_pos_v.resize(size_vehicles_v);

    random_pos_h.insert(random_pos_h.end(), random_pos_v.begin(), random_pos_v.end());
    random_pos.insert(random_pos.end(), random_pos_h.begin(), random_pos_h.end());

    random_shuffle(random_pos.begin(), random_pos.end(), myrandom);//Coloca en ordena aleatorio las posiciones iniciales

    i = 0;
    int size_autonomous = percentage_autonomous * size_vehicles;
    while (i < size_vehicles) {

        vehicles_A[i] = random_pos[i];
        vehicles_Z[i] = random_pos[i];

        if (i < size_autonomous){
             vehicles_A[i].autonomous = true;
             vehicles_Z[i].autonomous = true;
        }

        SetValueCellStreet(vehicles_A[i].type_street, vehicles_A[i].position.y, vehicles_A[i].position.x, 1, true, i);

        i++;
    }

    //qDebug() << "Size:" << size_vehicles;
    //qDebug() << "Random size:" << random_pos.size();

    random_pos.clear();

    SwitchMatricesRW();
    SwitchVehiclesRW();

    return 0;
}

void SwitchVehiclesRW()
{

    int i;

    if (switch_vehicles == true){

        //Inicializa matriz de lectura
        pr_vehicles = vehicles_A;
        pw_vehicles = vehicles_Z;
    }
    else {

        //Inicializa matriz de lectura
        pr_vehicles = vehicles_Z;
        pw_vehicles = vehicles_A;

    }

    switch_vehicles = !switch_vehicles;
}

void resetCityWrite()
{
    int i;
    int m, n;

    for (n = 0; n < n_hor_streets; n++){
        for (i = 0; i < d_hor_street; i++){
            pw_horizontal_streets[n][i].value = 0;
            pw_horizontal_streets[n][i].id = 0;
            pw_horizontal_streets[n][i].visible = true;
        }
    }

    for (m = 0; m < m_ver_streets; m++){
        for (i = 0; i < d_ver_street; i++){
            pw_vertical_streets[m][i].value = 0;
            pw_vertical_streets[m][i].id = 0;
            pw_vertical_streets[m][i].visible = true;
        }
    }

    int x_h;
    int x_v;


    for (n = 0; n < n_hor_streets; n++)
        for (m = 0; m < m_ver_streets; m++){

            x_h = m * d_side_block + m;
            x_v = n * d_side_block + n;

            pw_intersections[n][m].value = 0;
            pw_intersections[n][m].id = 0;
            pw_intersections[n][m].visible = true;

            pw_horizontal_streets[n][x_h].value = 2;
            pw_vertical_streets[m][x_v].value = 2;

            pw_horizontal_streets[n][x_h].visible = false;
            pw_vertical_streets[m][x_v].visible = false;

        }

}

void printCity()
{

    int i, j;

    printf ("---Read----\n");

    for (i = 0; i < n_hor_streets; i++){
        for (j = 0; j < d_hor_street; j++)
            printf ("%d ", pr_horizontal_streets[i][j]);
        printf ("\n");
    }
    printf ("\n");

    for (j = 0; j < d_ver_street; j++){
        for (i = 0; i < m_ver_streets; i++)
            printf ("%d ", pr_vertical_streets[i][j]);
        printf ("\n");
    }
    printf ("\n");

    printf ("---Write----\n");

    for (i = 0; i < n_hor_streets; i++){
        for (j = 0; j < d_hor_street; j++)
            printf ("%d ", pw_horizontal_streets[i][j]);
        printf ("\n");

    }
    printf ("\n");

    for (j = 0; j < d_ver_street; j++){
        for (i = 0; i < m_ver_streets; i++)
            printf ("%d ", pw_vertical_streets[i][j]);
        printf ("\n");
    }
    printf ("\n");
}


void SwitchMatricesRW()
{

    int i;

    if (switch_matriz == true){

        //Inicializa matriz de lectura
        for (i = 0; i < n_hor_streets; i++)
            pr_horizontal_streets[i] = horizontal_streets_A[i];

        for (i = 0; i < m_ver_streets; i++)
            pr_vertical_streets[i] = vertical_streets_A[i];

        for (i = 0; i < n_hor_streets; i++)
            pr_intersections[i] = intersections_A[i];

        //Inicializa matriz de escritura
        for (i = 0; i < n_hor_streets; i++)
            pw_horizontal_streets[i] = horizontal_streets_Z[i];

        for (i = 0; i < m_ver_streets; i++)
            pw_vertical_streets[i] = vertical_streets_Z[i];

        for (i = 0; i < n_hor_streets; i++)
            pw_intersections[i] = intersections_Z[i];
    }
    else {

        //Inicializa matriz de lectura
        for (i = 0; i < n_hor_streets; i++)
            pr_horizontal_streets[i] = horizontal_streets_Z[i];

        for (i = 0; i < m_ver_streets; i++)
            pr_vertical_streets[i] = vertical_streets_Z[i];

        for (i = 0; i < n_hor_streets; i++)
            pr_intersections[i] = intersections_Z[i];

        //Inicializa matriz de escritura
        for (i = 0; i < n_hor_streets; i++)
            pw_horizontal_streets[i] = horizontal_streets_A[i];

        for (i = 0; i < m_ver_streets; i++)
            pw_vertical_streets[i] = vertical_streets_A[i];

        for (i = 0; i < n_hor_streets; i++)
            pw_intersections[i] = intersections_A[i];

    }

    switch_matriz = !switch_matriz;

}

bool GetVehiclesStopped(char type_street, int y, int x)
{

    int id;

    if (GetValueCellStreet(type_street, y, x) == 1) {

        id = GetIDCellStreet(type_street, y, x);

        if (GetTypeStreetVehicle(id) == type_street)
            if (GetVelocityVehicle(id) == 0)
                return true;

    }

    return false;

}

bool isVehiclesStoppedIntersection(int n, int m)
{
   int x, y;

   y = n;
   x =  m * d_side_block + m;

   if (GetVehiclesStopped('H', y, x) == true)
       return true;

   y = m;
   x = n * d_side_block + n;

    if (GetVehiclesStopped('V', y, x) == true)
        return true;

    return false;
}

bool VehiclesStoppedDistance_e(char type_street, int n, int m, int d_e)
{
    int x, y;
    int direction;
    int e;

    if (type_street == 'H') {

        y = n;
        direction = (n % 2) == 0 ? 'R' : 'L';

        if (direction == 'R') {

            for (e = d_e; e > 0; e--) {
                x = (m * d_side_block + m) + e;

                if (GetVisibleCellStreet('H', n, x) == true){
                    if (GetVehiclesStopped(type_street, y, x) == true)
                        return true;
                }
            }
        }
        else {

            if (m > 0) {
                for (e = d_e; e > 0; e--) {
                    x = (m * d_side_block + m) - e;

                    if (GetVisibleCellStreet('H', n, x) == true){

                        if (GetVehiclesStopped(type_street, y, x) == true)
                            return true;
                    }
                }
            }
            else {

                for (e = d_e; e > 0; e--) {
                    x = d_hor_street - e;

                    if (GetVisibleCellStreet('H', n, x) == true){

                        if (GetVehiclesStopped(type_street, y, x) == true)
                            return true;
                    }
                }
            }
        }
    } else if (type_street == 'V') {

        y = m;
        direction = (m % 2) == 0 ? 'R' : 'L';

        if (direction == 'R') {

            for (e = d_e; e > 0; e--) {
                x = (n * d_side_block + n) + e;

                if (GetVisibleCellStreet('V', m, x) == true){

                    if (GetVehiclesStopped(type_street, y, x) == true)
                        return true;
                }
            }
        }
        else {

            if (n > 0) {
                for (e = d_e; e > 0; e--) {
                    x = (n * d_side_block + n) - e;

                    if (GetVisibleCellStreet('V', m, x) == true){

                        if (GetVehiclesStopped(type_street, y, x) == true)
                            return true;
                    }
                }
            }
            else {
                for (e = d_e; e > 0; e--) {

                    x = d_ver_street - e;

                    if (GetVisibleCellStreet('V', m, x) == true){

                        if (GetVehiclesStopped(type_street, y, x) == true)
                            return true;
                    }
                }
            }
        }
    }

    return false;

}

int distance2(int position_1, int position_2, char dir, int distance_street)
{

    int d;

    if (position_1 < 0 || position_1 >= distance_street)
        position_1 = 0;

    if (position_2 < 0 || position_2 >= distance_street)
        position_2 = 0;

    if (position_1 == position_2){

       // qDebug() << "Esta en el semaforo " << position_1;
        return 0;
    }

    if (dir == 'R') {

        if (position_1 < position_2){
            d = (position_2 - position_1) - 1;
            //qDebug() << "C1N: " << n << "pn1 - pn - 1: " << GetPositionVehicle(n1) << " - " << GetPositionVehicle(n) << " = " << d;
        }
        else {
            d = (distance_street + position_2) - position_1 - 1;
            //qDebug() << "C2P: " << n << "d = (l - 1) - pn - p0" << n << " : " << length_h - 1 << " - " << GetPositionVehicle(n) << " + " << GetPositionVehicle(n1) << " = " << d;
        }

    } else {

        if (position_2 < position_1){
            d = (position_1 - position_2) - 1;
            //qDebug() << "C1N: " << n << "pn1 - pn - 1: " << GetPositionVehicle(n1) << " - " << GetPositionVehicle(n) << " = " << d;
        }
        else {

            d = (distance_street + position_1) - position_2 - 1;
            //qDebug() << "C2P: " << n << "d = (l - 1) - pn - p0" << n << " : " << length_h - 1 << " - " << GetPositionVehicle(n) << " + " << GetPositionVehicle(n1) << " = " << d;
        }
    }

    return d;
}

int CalculateVelocity(unsigned int dn, int vn, int vp)
{
    int vn_new = 0;

    //Acceleration
    if (dn >= acc[vn][vp]){
        if (frand() <= Ra[vn]){ //Acelera
            int tmp_vn = vn + delta_v;
            vn_new = min(tmp_vn, vmax);
        }
        else
            vn_new = vn;
    }
    else if (dn < acc[vn][vp] && dn >= keep[vn][vp] ){//Random slowing
        if (frand() <= Rs){
            int tmp_vn = vn - delta_v;
            vn_new = max(tmp_vn, 0);
        }
        else
            vn_new = vn;
    }
    else if (dn < keep[vn][vp] && dn >= dec0[vn][vp] && vn > 0){//breaking
        int tmp_vn = vn - delta_v;
        vn_new = max(tmp_vn, 0);
    }
    else if (dn < dec0[vn][vp] && vn > 0){//emergency breaking
        int tmp_vn = vn - M;
        vn_new = max(tmp_vn, 0);
    }

    return vn_new;
}

int CalculateVelocityAutonomous(unsigned int dn, int vn, int vp)
{
    int vn_new = 0;

    //Acceleration
    if (dn >= acc[vn][vp]){
        //Siempre acelera
        int tmp_vn = vn + delta_v;
        vn_new = min(tmp_vn, vmax);
    }
    else if (dn < acc[vn][vp] && dn >= keep[vn][vp] ){//Random slowing
       //Mantiene velocidad y nunca sobre frena
        vn_new = vn;
    }
    else if (dn < keep[vn][vp] && dn >= dec0[vn][vp] && vn > 0){//breaking
        int tmp_vn = vn - delta_v;
        vn_new = max(tmp_vn, 0);
    }
    else if (dn < dec0[vn][vp] && vn > 0){//emergency breaking
        int tmp_vn = vn - M;
        vn_new = max(tmp_vn, 0);
    }

    return vn_new;
}


int CalculateVelocityTwo(unsigned int dn, int vn)
{
    int vn_new = 0;

    //Acceleration
    if (dn >= acc[vn][0]){
        if (frand() <= Ra[vn]){ //Acelera
            int tmp_vn = vn + delta_v;
            vn_new = min(tmp_vn, vmax);
        }
        else
            vn_new = vn;
    }
    else if (dn < acc[vn][0] && dn >= keep[vn][0] ){//Random slowing
        if (frand() <= Rs){
            int tmp_vn = vn - delta_v;
            vn_new = max(tmp_vn, 0);
        }
        else
            vn_new = vn;
    }
    else if (dn < keep[vn][0] && dn >= dec0[vn][0] && vn > 0){//breaking
        int tmp_vn = vn - delta_v;
        vn_new = max(tmp_vn, 0);
    }
    else if (dn < dec0[vn][0] && dn >= dbreak[vn] && vn > 0){//emergency breaking

        int tmp_vn = vn - M;
        vn_new = max(tmp_vn, 0);

    }
    else if (dn < dbreak[vn]){ //Predice qu no puede detenerse antes del semaforo
        vn_new = vn; // continua con la misma velocidad para pasarse el amarillo o rojo
    }


    return vn_new;
}

int CalculateVelocityTwoAutonomous(unsigned int dn, int vn)
{
    int vn_new = 0;

    //Acceleration
    if (dn >= acc[vn][0]){
        //siempre Acelera
        int tmp_vn = vn + delta_v;
        vn_new = min(tmp_vn, vmax);

    }
    else if (dn < acc[vn][0] && dn >= keep[vn][0] ){//Random slowing
        //nunca sobre frena
        vn_new = vn;
    }
    else if (dn < keep[vn][0] && dn >= dec0[vn][0] && vn > 0){//breaking
        int tmp_vn = vn - delta_v;
        vn_new = max(tmp_vn, 0);
    }
    else if (dn < dec0[vn][0] && dn >= dbreak[vn] && vn > 0){//emergency breaking

        int tmp_vn = vn - M;
        vn_new = max(tmp_vn, 0);

    }
    else if (dn < dbreak[vn]){ //Predice qu no puede detenerse antes del semaforo
        vn_new = vn; // continua con la misma velocidad para pasarse el amarillo o rojo
    }


    return vn_new;
}

void RunSimulationGreenWave(int tick)
{

    int i;

    SPosition pos;
    char type_street;
    char direction;

    int x, y;
    int v;

    SPosition pos_front;
    char type_street_front;
    //char direction_front;

    int xp;
    int vp;

    int n, m;
    int d_street;

    unsigned int dn;
    int vn_new, vn_new_ligth;

    resetCityWrite();

    for (i = 0; i < size_vehicles; i++) {

        pos = GetPositionVehicle(i);
        x = pos.x;
        y = pos.y;
        v = GetVelocityVehicle(i);
        type_street = GetTypeStreetVehicle(i);
        direction = GetDirectionVehicle(i);

        int pos_t = 0;
        pos_t = GetRegionTrafficLight(type_street, x, direction);

        if (type_street == 'H') {
            n = y;
            m = pos_t;
            d_street = d_hor_street;
        }
        else {
            n = pos_t;
            m = y;
            d_street = d_ver_street;
        }

        int id_f = lookFrontID(type_street, direction, y, x);

        if (id_f != -1){
            type_street_front = GetTypeStreetVehicle(id_f);

            if (type_street_front == type_street){
                pos_front = GetPositionVehicle(id_f);
                xp = pos_front.x;
                vp = GetVelocityVehicle(id_f);
                //qDebug() << id_f << direction << x << "-" << xp;
            }

            else {

                int pos_intersection = GetPositionIntersectionTrafficLight(type_street, n, m);

                if (GetValueCellStreet(type_street, y, pos_intersection) == 1 )
                    xp = pos_intersection;
                else
                    xp = pos_front.x;

                vp = 0;
            }
        }
        else {

            if (direction == 'R')
                xp = (x == 0) ? d_street - 1 : x - 1;
            else
                xp = (x == d_street - 1) ? 0 : x + 1;

            vp = vmax;
            //qDebug() << "No hay frontal" << id_f << direction << x << "-" << xp;
        }


        //Distancia respecto al vehiculo
        dn = distance2(x, xp, direction, d_street);

        if (GetAutonomousVehicle(i) == true)
           vn_new = CalculateVelocityAutonomous(dn, v, vp);
        else
           vn_new = CalculateVelocity(dn, v, vp);

        //qDebug() << vn_new;

        bool block_intersection = false;

        if (behavior_green == true) {

            if (GetAutonomousVehicle(i) == true) {
                if (VehiclesStoppedDistance_e(type_street, n, m, 4) == true)
                    block_intersection = true;
            }
        }

        if (GetValueTrafficLight(type_street, n, m) == 0 || block_intersection == true) {//Distancia respecto al semaforo en rojo

            dn = distance2(x, GetPositionTrafficLight(type_street, n, m), direction, d_street);

            if (GetAutonomousVehicle(i) == true)
              vn_new_ligth = CalculateVelocityTwoAutonomous(dn, v);
            else
              vn_new_ligth = CalculateVelocityTwo(dn, v);//vp es siempre 0 porque el semaforo tiene velocidad 0

            vn_new = min(vn_new, vn_new_ligth);
        }


        if (direction == 'R') {//direccion derecha

            if ((x + vn_new) >= d_street)
                x = (x + vn_new) % d_street;
            else
                x = x + vn_new;
        }
        else if (direction == 'L') {//direccion izquierda

            if ((x - vn_new) < 0)
                x = d_street + ((x - vn_new) % d_street);
            else
                x = x - vn_new;
        }

        SetTypeStreetVehicle(type_street, i);
        SetPositionVehicle(type_street, y, x, i);
        SetVelocityVehicle(vn_new, i);
        SetDirectionVehicle(direction, i);
        SetVisibleVehicle(i, true);

        if (frand() < p_turn) {
            turn(type_street, y, x, vn_new, direction, true, i);
        }

        SetValueCellStreet(type_street, y, x, 1, true, i);

    }

    for (n = 0; n < n_hor_streets; n++)
        for (m = 0; m < m_ver_streets; m++)
            RunTrafficLight(n, m);

   //printf("Colision: %d \n", collisions);
   // qDebug() << "Colision: " << collisions;

    if (tick < n_ticks) {
        Velocity();
        Flux(density);
    }

    SwitchMatricesRW();
    SwitchVehiclesRW();

    //qDebug() << "Run simulation";
}


int lookFrontID(char type_street, char direction, int y, int x)
{
    int id_f;
    int k;
    int limit;

    id_f = -1;

    if (type_street == 'H')
        limit = d_hor_street;
    else
        limit = d_ver_street;

    if (direction == 'R'){
        k = x + 1;
        while (k < limit) {

            if (GetValueCellStreet(type_street, y, k) == 1)
                return GetIDCellStreet(type_street, y, k);
            else
                k++;
        }

        k = 0;
        while (k < x) {

            if (GetValueCellStreet(type_street, y, k) == 1)
                return GetIDCellStreet(type_street, y, k);
            else
                k++;
        }
    }
    else {

        k = x - 1;
        while (k >= 0) {

            if (GetValueCellStreet(type_street, y, k) == 1)
                return GetIDCellStreet(type_street, y, k);
            else
                k--;
        }

        k = limit - 1;
        while (k > x) {

            if (GetValueCellStreet(type_street, y, k) == 1)
                return GetIDCellStreet(type_street, y, k);
            else
                k--;
        }
    }


    return id_f;
}


bool turn(char &type_street, int &y, int &x, int &v, char &direction, bool visible, int id)
{

        if (type_street == 'H'){

            if (y < 0 || y >= n_hor_streets) {
                printf ("Error en H, y = ", y);
                return false;
            }

            if (x < 0 || x >= d_hor_street) {
                printf ("Error en H, x = ", x);
                return false;
            }

            if (pr_horizontal_streets[y][x].value == 2) {

                int y_new = x / (1 + d_side_block);
                int x_new = y * d_side_block + y;

                y = y_new;
                x = x_new;

                type_street = 'V';
                SetTypeStreetVehicle('V', id);

                SetPositionVehicle(type_street, y, x, id);

                if ((y % 2) == 0)
                   direction = 'R';
                else
                   direction = 'L';

                SetDirectionVehicle(direction, id);

                //v = Rule_184(type_street, y, x, direction);
                SetVelocityVehicle(v, id);

                SetVisibleVehicle(id, visible);

                return true;
           }
        }


        else if (type_street == 'V'){

            if (y < 0 || y >= m_ver_streets) {
                printf ("Error en V, y = ", y);
                return false;
            }

            if (x < 0 || x >= d_ver_street) {
                printf ("Error en V, x = ", x);
                return false;
            }

            if (pr_vertical_streets[y][x].value == 2) {

                int y_new = x / (1 + d_side_block);
                int x_new = y * d_side_block + y;

                y = y_new;
                x = x_new;

                type_street = 'H';
                SetTypeStreetVehicle('H', id);

                SetPositionVehicle(type_street, y, x, id);

                if ((y % 2) == 0)
                   direction = 'R';
                else
                   direction = 'L';

                SetDirectionVehicle(direction, id);

                //v = Rule_184(type_street, y, x, direction);
                SetVelocityVehicle(v, id);

                SetVisibleVehicle(id, visible);

                return true;

            }
        }

        return false;
}


void RunSimulationSelfOrganization(int tick)
{

    int i;

    SPosition pos;
    char type_street;
    char direction;

    int x, y;
    int v;

    SPosition pos_front;
    char type_street_front;
    //char direction_front;

    bool prev_visible;
    bool visible;

    int xp;
    int vp;

    int n, m;
    int d_street;

    unsigned int dn;
    int vn_new, vn_new_ligth;

    resetCityWrite();

    for (i = 0; i < size_vehicles; i++) {

        pos = GetPositionVehicle(i);
        x = pos.x;
        y = pos.y;
        v = GetVelocityVehicle(i);
        type_street = GetTypeStreetVehicle(i);
        direction = GetDirectionVehicle(i);
        prev_visible = GetVisibleVehicle(i);

        int pos_t = 0;
        pos_t = GetRegionTrafficLightSO(type_street, x, direction);

        if (type_street == 'H') {
            n = y;
            m = pos_t;
            d_street = d_hor_street;
        }
        else {
            n = pos_t;
            m = y;
            d_street = d_ver_street;
        }

        int id_f = lookFrontID(type_street, direction, y, x);

        if (id_f != -1){
            type_street_front = GetTypeStreetVehicle(id_f);

            if (type_street_front == type_street){
                pos_front = GetPositionVehicle(id_f);
                xp = pos_front.x;
                vp = GetVelocityVehicle(id_f);
                //qDebug() << id_f << direction << x << "-" << xp;
            }

            else {

                int pos_intersection = GetPositionIntersectionTrafficLightSO(type_street, n, m);

                if (GetValueCellStreet(type_street, y, pos_intersection) == 1)
                    xp = pos_intersection;
                else
                    xp = pos_front.x;

                vp = 0;
            }
        }
        else {

            if (direction == 'R')
                xp = (x == 0) ? d_street - 1 : x - 1;
            else
                xp = (x == d_street - 1) ? 0 : x + 1;

            vp = vmax;
            //qDebug() << "No hay frontal" << id_f << direction << x << "-" << xp;
        }

        //Distancia respecto al vehiculo
        dn = distance2(x, xp, direction, d_street);
        vn_new = CalculateVelocity(dn, v, vp);

        //qDebug() << vn_new;

        if (GetValueTrafficLightSO(type_street, n, m) == 0) {//Distancia respecto al semaforo en rojo

            dn = distance2(x, GetPositionTrafficLightSO(type_street, n, m), direction, d_street);
            vn_new_ligth = CalculateVelocityTwo(dn, v);//vp es siempre 0 porque el semaforo tiene velocidad 0
            vn_new = min(vn_new, vn_new_ligth);
        }

        if (direction == 'R') {//direccion derecha

            if ((x + vn_new) >= d_street)
                x = (x + vn_new) % d_street;
            else
                x = x + vn_new;
        }
        else if (direction == 'L') {//direccion izquierda

            if ((x - vn_new) < 0)
                x = d_street + ((x - vn_new) % d_street);
            else
                x = x - vn_new;
        }

        visible = determineVisible(type_street, direction, x, prev_visible);//Visibilidad

        SetTypeStreetVehicle(type_street, i);
        SetPositionVehicle(type_street, y, x, i);
        SetVelocityVehicle(vn_new, i);
        SetDirectionVehicle(direction, i);
        SetVisibleVehicle(i, visible);

        if (frand() < p_turn) {
            turn(type_street, y, x, vn_new, direction, visible, i);
        }

        SetValueCellStreet(type_street, y, x, 1, true, i);

    }

    for (n = 0; n < n_hor_streets; n++)
        for (m = 0; m < m_ver_streets; m++)
            RunTrafficLight(n, m);

   //printf("Colision: %d \n", collisions);
   // qDebug() << "Colision: " << collisions;

    if (tick < n_ticks) {
        Velocity();
        Flux(density);
    }

    SwitchMatricesRW();
    SwitchVehiclesRW();

    //qDebug() << "Run simulation";

}

int mainFunction(int n_h, int n_v, int n_b, float p_t, int met, float _P, int max_n, int max_m, int min_t, int max_t, int met_s, float pre, int dis_d, int dis_r, int dis_e, float per_auto)
{

    float density;

    fp_f = fopen("measuresflu.csv", "w");   // Abrir archivo para escritura
    fp_v = fopen("measuresvel.csv", "w");   // Abrir archivo para escritura

    fp_fopt = fopen("fluopt.csv", "w");   // Abrir archivo para escritura
    fp_vopt = fopen("velopt.csv", "w");   // Abrir archivo para escritura

    int tmp_ticks = 5400;//para estabilizar el sistema


    fprintf(fp_v, "%f,%f\n", 0.0, 0.0);//con estos datos se grafica el diagrama fundamental del trafico
    fprintf(fp_f, "%f,%f\n", 0.0, 0.0);//con estos datos se grafica el diagrama fundamental del trafico

    fprintf(fp_vopt, "%f,%f\n", 0.0, 0.0);
    fprintf(fp_fopt, "%f,%f\n", 0.0, 0.0);


    qDebug() << "Ejecuntando, espere por favor...";
    clock_t start = clock();

    float dens = 1.0 / (float) (n_h * (n_b * n_h) + n_h); //Se asume que la ciudad tiene el mismo tamanio horizontal y vertical para que la densidad sea la misma

    //qDebug() << dens << n_h << n_b;

    for (density = dens; density <= 1.0; density+=size_step){
        //printf("%f\n", density);
        save_velocity = 0;
        save_flux = 0;

        for (int i = 0; i < n_exp; i++) {

            switch_matriz = true;
            switch_vehicles = true;

            velocity = 0;
            flux = 0;
            velocity_total = 0;
            flux_total = 0;
            collisions = 0;

            InializedCity(n_h, n_v, n_b, density, density, p_t, per_auto);
            InializedTrafficLights(met, _P, max_n, max_m, min_t, max_t);
            InializedSensores(met_s, pre, dis_d, dis_r, dis_e);

            for (int tick = 0; tick < tmp_ticks; tick++)//Establizar sistema (superar trasciendes)
                RunSimulation(tick);

            velocity = 0;
            flux = 0;
            velocity_total = 0;
            flux_total = 0;
            collisions = 0;

            for (int tick = 0; tick < n_ticks; tick++)
                RunSimulation(tick);

            //qDebug() << "No. Colisiones: " << collisions << "- No. Vehiculos" << total_vehicles;
            CalculateSaveMeasures();

            qDebug() << "Density:" << density;

            freeSensors();
            freeSensorsTraditional();
            freeTrafficLightSO();
            freeTrafficLight();
            FreeCity();

        }

        SaveMeasures(density);
        //printf("Density: %f", densitiy);

        //printf("\a\a");
    }

    qDebug() << "Fin:";

    //printf ("End...\n ");
    printf("Tiempo transcurrido: %f", ((double)clock() - start) / CLOCKS_PER_SEC);

    fclose(fp_f);
    fclose(fp_v);
    fclose(fp_fopt);
    fclose(fp_vopt);

    return 0;
}


int mainFunctionAutonomous(int n_h, int n_v, int n_b, float p_t, int met, float _P, int max_n, int max_m, int min_t, int max_t, int met_s, float pre, int dis_d, int dis_r, int dis_e, float per_auto)
{

    float density;

    fp_f = fopen("Automeasuresflu.csv", "w");   // Abrir archivo para escritura
    fp_v = fopen("Automeasuresvel.csv", "w");   // Abrir archivo para escritura


    fp_fopt = fopen("fluopt.csv", "w");   // Abrir archivo para escritura
    fp_vopt = fopen("velopt.csv", "w");   // Abrir archivo para escritura

    int tmp_ticks = 5400;//para estabilizar el sistema

    qDebug() << "Ejecuntando, espere por favor...";
    clock_t start = clock();

    //float dens = 1.0 / (float) (n_h * (n_b * n_h) + n_h); //Se asume que la ciudad tiene el mismo tamanio horizontal y vertical para que la densidad sea la misma

    //qDebug() << dens << n_h << n_b;

    fprintf(fp_f, "%s,%s,%s\n", "Densidad", "Pocentaje", "Flujo");
    fprintf(fp_v, "%s,%s,%s\n", "Densidad", "Pocentaje", "Velocidad");


    float per_autonomous = 0.0;
    for (int k = 0; k <= 10; k++){

        fprintf(fp_v, "%f,%f,%f\n", 0.0, per_autonomous, 0.0);//con estos datos se grafica el diagrama fundamental del trafico
        fprintf(fp_f, "%f,%f,%f\n", 0.0, per_autonomous, 0.0);//con estos datos se grafica el diagrama fundamental del trafico

        density = 0.1;
        for (int j = 1; j <= 10; j++){
            //printf("%f\n", density);
            save_velocity = 0;
            save_flux = 0;

            for (int i = 0; i < n_exp; i++) {

                switch_matriz = true;
                switch_vehicles = true;

                velocity = 0;
                flux = 0;
                velocity_total = 0;
                flux_total = 0;
                collisions = 0;

                InializedCity(n_h, n_v, n_b, density, density, p_t, per_autonomous);
                InializedTrafficLights(met, _P, max_n, max_m, min_t, max_t);
                InializedSensores(met_s, pre, dis_d, dis_r, dis_e);

                for (int tick = 0; tick < tmp_ticks; tick++)//Establizar sistema (superar trasciendes)
                    RunSimulation(tick);

                velocity = 0;
                flux = 0;
                velocity_total = 0;
                flux_total = 0;
                collisions = 0;

                for (int tick = 0; tick < n_ticks; tick++)
                    RunSimulation(tick);

                //qDebug() << "No. Colisiones: " << collisions << "- No. Vehiculos" << total_vehicles;
                CalculateSaveMeasures();

                qDebug() << "Density:" << density << "Autonomous:" << per_autonomous;

                freeSensors();
                freeSensorsTraditional();
                freeTrafficLightSO();
                freeTrafficLight();
                FreeCity();

            }

            SaveMeasuresAutonomous(density, per_autonomous);

            //printf("Density: %f", densitiy);
            density+=0.1;
        }

        per_autonomous+=0.1;
    }

    //printf("\a\a");
    qDebug() << "Fin:";

    //printf ("End...\n ");
    printf("Tiempo transcurrido: %f", ((double)clock() - start) / CLOCKS_PER_SEC);

    fclose(fp_f);
    fclose(fp_v);
    fclose(fp_fopt);
    fclose(fp_vopt);


    return 0;
}

