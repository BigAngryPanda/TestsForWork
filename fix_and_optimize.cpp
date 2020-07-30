/*
	Fix and optimize code

	Programm has to define order of car moving through crossroad

	If there is another car from right near the first car then the first car must pass the other car
	If we can't decide which car must pass (i.g. 4 cars are close enough) then car with lowest x coordinate is moving
	Cars must not intersect
*/

/*
	There is previous code in comments (if I only add something there will be empty comments)
*/

/*
*/
#include <cstdlib>
#include <vector>
#include <ctime>

struct sPos {
	sPos() { x = 0; y = 0; }
	sPos(int aX, int aY) { x = aX; y = aY; }

	int x;
	int y;

	bool operator <(const sPos& otherPos)
	{
		return x > otherPos.x;
	}
};

struct sSize
{
	sSize() { width = 0; height = 0; }
	sSize(int aW, int aH) { width = aW; height = aW; }
	
	int width;
	int height;
};

struct sRect
{
	sRect() {};
//	sRect(int x, int y, int w, int h) { pos.x = x; pos.y = y; size.width = w; size.height = h; }
	sRect(int x, int y, int w, int h) { pos = sPos(x, y); size = sSize(w, h); }
	sPos pos;
	sSize size;

	bool intersects(const sRect& other) {
		return !((other.pos.x + other.size.width <= pos.x)  ||
				 (other.pos.y + other.size.height <= pos.y) ||
				 (other.pos.x >= pos.x + size.width)        ||
				 (other.pos.y >= pos.y + size.height));
	}

	bool operator <(const sRect& otherRect)
	{
		return this->pos < otherRect.pos;
	}
};

enum class eDirection {
	UP,
	LEFT,
	RIGHT,
	DOWN
};

struct sCar {
	sRect rect;
	eDirection dir;
	int speed;

	void move() {
		switch (dir) {
//		case eDirection::up:
		case eDirection::UP:
			rect.pos.y += speed;
		case eDirection::DOWN:
			rect.pos.y -= speed;
		case eDirection::RIGHT:
			rect.pos.x += speed;
		case eDirection::LEFT:
			rect.pos.x -= speed;
		}
	}

/*
	sRect getFuturePos() {
		switch (dir) {
			case eDirection::up:
				return sRect(rect.pos.x, rect.pos.y + speed, rect.size.width, rect.size.height);
			case eDirection::DOWN:
				return sRect(rect.pos.x, rect.pos.y - speed, rect.size.width, rect.size.height);
			case eDirection::RIGHT:
				return sRect(rect.pos.x + speed, rect.pos.y, rect.size.width, rect.size.width);
			case eDirection::LEFT:
				return sRect(rect.pos.x + speed, rect.pos.y, rect.size.width, rect.size.height);
		}
	}
*/

	sRect getFuturePos() {
		sRect result;

		switch (dir) {
			case eDirection::UP:
				result = sRect(rect.pos.x, rect.pos.y + rect.size.height*speed, rect.size.width, rect.size.height);
				break;
			case eDirection::DOWN:
				result = sRect(rect.pos.x, rect.pos.y - rect.size.height*speed, rect.size.width, rect.size.height);
				break;
			case eDirection::RIGHT:
				result = sRect(rect.pos.x + rect.size.width*speed, rect.pos.y, rect.size.width, rect.size.height);
				break;
			case eDirection::LEFT:
				result = sRect(rect.pos.x - rect.size.width*speed, rect.pos.y, rect.size.width, rect.size.height);
				break;
		}

		return result;
	}

/*
	bool needPassOtherCar(sCar* otherCar) {
		bool result;

		switch (dir) {
			case eDirection::up:
				auto otherdir = otherCar->dir;
				if (otherdir == eDirection::LEFT)
					result = true;
				break;
			case eDirection::DOWN:
				auto otherdir = otherCar->dir;
				if (otherdir == eDirection::RIGHT)
					result = true;
				break;
			case eDirection::RIGHT:
				auto otherdir = otherCar->dir;
				if (otherdir == eDirection::UP)
					result = true;
				break;
			case eDirection::LEFT:
				auto otherdir = otherCar->dir;
				if (otherdir == eDirection::LEFT)
					result = false;
				else
					result = false;
				break;
		}

		return result;
	}
*/
	bool needPassByDirection(sCar* otherCar) {
		bool result = false;

		auto otherdir = otherCar->dir;

		switch (dir) {
			case eDirection::UP:
				if (otherdir == eDirection::LEFT)
					result = true;
				break;
			case eDirection::DOWN:
				if (otherdir == eDirection::RIGHT)
					result = true;
				break;
			case eDirection::RIGHT:
				if (otherdir == eDirection::UP)
					result = true;
				break;
			case eDirection::LEFT:
				if (otherdir == eDirection::DOWN)
					result = true;
				break;
		}

		return result;
	}

	bool needPassOtherCar(sCar* otherCar) {
		return needPassByDirection(otherCar) && 
			   getFuturePos().intersects(otherCar->getFuturePos());
	}

	bool isOutOfScreen(int screen_width, int screen_height) {
		return rect.pos.x <= 0            || 
			   rect.pos.x >= screen_width || 
			   rect.pos.y <= 0            || 
			   rect.pos.y >= screen_height;
	}

	bool operator <(const sCar& otherCar)
	{
		return this->rect < otherCar.rect;
	}

	virtual int getFuel() = 0;
	virtual void refill(int count) = 0;
	virtual ~sCar() {};
};

// struct sGasEngine : sCar {
struct sGasEngine : virtual sCar {
	int getFuel() { return fuel; }
	void refill(int count) { fuel += count; }
	void move() { fuel--; sCar::move(); }

	int fuel;
};

// struct sElectroCar : sCar {
struct sElectroCar : virtual sCar {
	int getFuel() { return charge; }
	void refill(int count) { charge += count; }
	void move() { charge--; sCar::move(); }

	int charge;
};

struct sHybrid : sGasEngine, sElectroCar {
//	void refill(int count) { charge += count / 2; fuel += count / 2; }
	void refill(int count) { charge = charge + count / 2 + count % 2; fuel += count / 2; }
	int getFuel() { return charge + fuel; }

/*
	void move() {
		if (rand() % 2 == 0)
		{
			charge--;
		}
		else
		{
			fuel--;
		}

		sCar::move();
	}
*/

	void move() {
		if (rand() % 2 == 0)
		{
			sElectroCar::move();
			charge--;
		}
		else
		{
			sGasEngine::move();
			fuel--;
		}
	}
};


std::vector<sCar*> asdasd;
const int initialCarsCount = 10;

#define SCREEN_WIDTH 1024
#define SCREEN_HEIGHT 768

//
const int default_width = 100;
const int default_height = 100;

/*
void spawnCarFromTop() {
	sCar* car;
	int carType = rand();

	if (carType % 3 == 0) {
		car = new sGasEngine();
	}
	else if (carType % 3 == 1) {
		car = new sElectroCar();
	}
	else if (carType % 3 == 2) {
		car = new sHybrid();
	}

	car->rect = sRect(SCREEN_WIDTH / 2, 0, 100, 100);
	car->speed = 1;
	car->dir = eDirection::DOWN;
}

void spawnCarFromBot() {
	sCar* car;
	int carType = rand();

	if (carType % 3 == 0) {
		car = new sGasEngine();
	}
	else if (carType % 3 == 1) {
		car = new sElectroCar();
	}
	else if (carType % 3 == 2) {
		car = new sHybrid();
	}
	car->rect = sRect(SCREEN_WIDTH / 2, SCREEN_HEIGHT, 100, 100);
	car->speed = 1;
}

void SpawnCarFromLeft() {
	sCar* car;
	int carType = rand();

	if (carType % 3 == 0) {
		car = new sGasEngine();
	}
	else if (carType % 3 == 1) {
		car = new sElectroCar();
	}
	else if (carType % 3 == 2) {
		car = new sHybrid();
	}

	car->rect = sRect(0, SCREEN_HEIGHT / 2, 100, 100);
	car->speed = 1;
}

void spawnCarFromRight() {
	sCar* car;
	int carType = rand();

	if (carType % 3 == 0) {
		car = new sGasEngine();
	}
	else if (carType % 3 == 1) {
		car = new sElectroCar();
	}
	else if (carType % 3 == 2) {
		car = new sHybrid();
	}

	car->rect = sRect(0, SCREEN_HEIGHT / 2, 100, 100);
	car->speed = 1;
}

void spawnCar() {
	if (rand() % 4 == 1)
		spawnCarFromRight();
	else if (rand() % 4 == 2)
		spawnCarFromTop();
	else if (rand() % 4 == 3)
		spawnCarFromBot();
	else if (rand() % 4 == 4)
		SpawnCarFromLeft();
}
*/

eDirection getDirection() {
	// C++17
	return eDirection(rand() % 4);
}

sCar* getCarType() {
	switch (rand() % 3)
	{
		case 0:
			return new sGasEngine();
		case 1:
			return new sElectroCar();
		default:
			return new sHybrid();
	}
}

sCar* spawnCar() {
	eDirection dir = getDirection();
	sCar* car = getCarType();

	switch (dir) {
		case eDirection::UP:
			car->rect = sRect(SCREEN_WIDTH / 2, 0, default_width, default_height);
			break;
		case eDirection::DOWN:
			car->rect = sRect(SCREEN_WIDTH / 2 - default_width, SCREEN_HEIGHT, default_width, default_height);
			break;
		case eDirection::RIGHT:
			car->rect = sRect(0, SCREEN_HEIGHT / 2 - default_height, default_width, default_height);
			break;
		case eDirection::LEFT:
			car->rect = sRect(SCREEN_WIDTH, SCREEN_HEIGHT / 2, default_width, default_height);
			break;
	}

	car->speed = 1;
	car->dir = dir;

	return car;
}
/*
bool main_loop() {
	for (auto car : asdasd) {
		for (auto car22 : asdasd) {
			if (car->getFuturePos().intersects(car22->getFuturePos())) {
				if (car->needPassOtherCar(car22))
					car->move();
			}
			else {
				car22->move();
			}
		}

		if (car->rect.pos.x <= 0 || car->rect.pos.x >= SCREEN_WIDTH || car->rect.pos.y <= 0 || car->rect.pos.y >= SCREEN_HEIGHT)
			spawnCar();
	}

	return main_loop();
}
*/

std::vector<sCar*>::iterator findMin() {
	std::vector<sCar*>::iterator result = asdasd.begin();

	for (std::vector<sCar*>::iterator i = result + 1; i != asdasd.end(); ++i)
	{
		if (*i < *result)
		{
			result = i;
		}
	}

	return result;
}

bool routeIsClear(std::vector<sCar*>::iterator car) {
	for (auto i = car + 1; i != asdasd.end(); ++i)
	{
		if ((*car)->needPassOtherCar(*i))
		{
			return false;
		}
	}

	return true;
}

void moveCars() {
	bool anyCarMoved = false;

	for (auto i = asdasd.begin(); i != asdasd.end(); ++i) {
		if (routeIsClear(i))
		{
			(*i)->move();
			anyCarMoved = true;
		}

		if ((*i)->isOutOfScreen(SCREEN_WIDTH, SCREEN_HEIGHT))
		{
			delete *i;
			*i = spawnCar();
		}
	}

	if (!anyCarMoved)
	{
		(*findMin())->move();
	}
}

void main_loop() {
	while (true) {
		moveCars();
	}
}

/*
int main(int argc, char** argv) {
	for (auto i = 0; i < initialCarsCount; ++i) {
		spawnCar();
	}

	main_loop();

	return 0;
}
*/
int main(int argc, char** argv) {
	srand(time(NULL));

	for (auto i = 0; i < initialCarsCount; ++i) {
		asdasd.push_back(spawnCar());
	}

	main_loop();

	return 0;
}