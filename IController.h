#ifndef ICONTROLLER_H
#define ICONTROLLER_H

class IController {
public:
	virtual void Tick(float deltaTime) = 0;
};

#endif
