"use strict";
class BodyParameters {
    constructor(acceleration = new DOMPoint(), velocity = new DOMPoint(), baseVelocity = new DOMPoint(), movement = new DOMPoint()) {
        this.acceleration = acceleration;
        this.velocity = velocity;
        this.baseVelocity = baseVelocity;
        this.movement = movement;
    }
    get fullVelocity() {
        return new DOMPoint(this.baseVelocity.x + this.velocity.x, this.baseVelocity.y + this.velocity.y);
    }
}
class ReadonlyBodyParameters {
    constructor(acceleration = new DOMPoint(), velocity = new DOMPoint(), baseVelocity = new DOMPoint(), movement = new DOMPoint()) {
        this.acceleration = acceleration;
        this.velocity = velocity;
        this.baseVelocity = baseVelocity;
        this.movement = movement;
    }
    get fullVelocity() {
        return new DOMPoint(this.baseVelocity.x + this.velocity.x, this.baseVelocity.y + this.velocity.y);
    }
}
class ObservableCollection {
    constructor() {
        this._collection = [];
    }
    get onChanged() {
        return this._onChanged;
    }
    get collection() {
        return this._collection;
    }
    set onChanged(value) {
        this._onChanged = value;
    }
    invokeChangedEvent() {
        if (this._onChanged) {
            this._onChanged();
        }
    }
    add(value) {
        this._collection.push(value);
        this.invokeChangedEvent();
    }
    delete(index) {
        this._collection.splice(index, 1);
        this.invokeChangedEvent();
    }
}
class PhysicalBody {
    constructor(position, mass, parameters) {
        this._force = new DOMPoint();
        this._parameters = parameters;
        this._path = new ObservableCollection();
        this._position = position;
        this._mass = mass;
        this._path.onChanged = this.reducePath.bind(this);
    }
    get force() {
        return this._force;
    }
    get position() {
        return this._position;
    }
    get mass() {
        return this._mass;
    }
    get parameters() {
        return this._parameters;
    }
    get path() {
        return this._path.collection;
    }
    set position(position) {
        this._path.add(this._position);
        this._position = position;
    }
    set mass(mass) {
        this._mass = mass;
    }
    set parameters(parameters) {
        this._parameters = parameters;
    }
    reducePath() {
        if (this._path.collection.length > PhysicalBody.maxPathLength) {
            this._path.delete(0);
        }
    }
    addForce(force) {
        this._force.x += force.x;
        this._force.y += force.y;
    }
    setForce(force) {
        this._force = force;
    }
    clearForces() {
        this._force = new DOMPoint();
    }
}
PhysicalBody.maxPathLength = 255;
class Utils {
    static distance(point1, point2) {
        return Math.sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
    }
    static trapezeArea(base1, base2, height) {
        return (base1 + base2) * height / 2;
    }
}
class PhysicalEngine {
    constructor(bodyes) {
        this.G = 6.68e-11;
        this.maxTimeDelta = 10;
        this._bodyes = bodyes;
        this._timeOffset = Date.now();
        this._speed = 1;
    }
    get speed() {
        return this._speed;
    }
    get bodyies() {
        return this._bodyes;
    }
    set speed(value) {
        this._speed = value;
    }
    exists(body) {
        const index = this._bodyes.indexOf(body);
        return index != -1;
    }
    getBodyIndex(body) {
        const index = this._bodyes.indexOf(body);
        if (index != -1) {
            return index;
        }
        else {
            throw new Error("Can't find body");
        }
    }
    computeForce(influencedBody) {
        const formula = (mass1, mass2, distance) => {
            return this.G * mass1 * mass2 / (distance * distance);
        };
        const influencedBodyIndex = this.getBodyIndex(influencedBody);
        influencedBody.clearForces();
        for (let influencingBodyIndex = 0; influencingBodyIndex < this._bodyes.length; influencingBodyIndex++) {
            const influencingBody = this._bodyes[influencingBodyIndex];
            if (influencingBodyIndex != influencedBodyIndex) {
                const forceValue = formula(influencedBody.mass, influencingBody.mass, Utils.distance(influencedBody.position, influencingBody.position));
                const angle = Math.atan2(influencingBody.position.y - influencedBody.position.y, influencingBody.position.x - influencedBody.position.x);
                const forceVector = new DOMPoint(forceValue * Math.cos(angle), forceValue * Math.sin(angle));
                influencedBody.addForce(forceVector);
            }
        }
    }
    computeAcceleration(body) {
        if (this.exists(body)) {
            // F = ma
            return new DOMPoint(body.force.x / body.mass, body.force.y / body.mass);
        }
        else {
            throw new Error("Can't find body");
        }
    }
    computeVelocity(body, timeDelta, acceleration) {
        if (this.exists(body)) {
            // finds area (difference between antiderivatives) between current acceleration and last captured acceleration
            return new DOMPoint(Utils.trapezeArea(body.parameters.acceleration.x, acceleration.x, timeDelta), Utils.trapezeArea(body.parameters.acceleration.y, acceleration.y, timeDelta)); // first integral of acceleration
        }
        else {
            throw new Error("Can't find body");
        }
    }
    computeMovement(body, timeDelta, velocity) {
        if (this.exists(body)) {
            // finds area (difference between antiderivatives) between current velocity and last captured velocity
            return new DOMPoint(Utils.trapezeArea(body.parameters.velocity.x, velocity.x, timeDelta), Utils.trapezeArea(body.parameters.velocity.y, velocity.y, timeDelta));
        }
        else {
            throw new Error("Can't find body");
        }
    }
    update() {
        const timeDelta = (Date.now() - this._timeOffset) * this._speed;
        this._bodyes.forEach((body) => {
            for (let chunk = 0; chunk < Math.ceil(timeDelta / this.maxTimeDelta); chunk++) {
                const computedTimeDelta = Math.min(this.maxTimeDelta, timeDelta - chunk * this.maxTimeDelta);
                this.computeForce(body);
                const acceleration = this.computeAcceleration(body);
                const velocity = this.computeVelocity(body, computedTimeDelta / 1000, acceleration);
                const movement = this.computeMovement(body, computedTimeDelta / 1000, new DOMPoint(velocity.x + body.parameters.baseVelocity.x, velocity.y + body.parameters.baseVelocity.y));
                body.position = new DOMPoint(movement.x + body.position.x, movement.y + body.position.y);
                // area addition rule (S = S1 + S2);
                body.parameters = new BodyParameters(acceleration, new DOMPoint(body.parameters.velocity.x + velocity.x, body.parameters.velocity.y + velocity.y), body.parameters.baseVelocity, new DOMPoint(body.parameters.movement.x + movement.x, body.parameters.movement.y + movement.y));
            }
        });
        this._timeOffset = Date.now();
    }
    resetTimeOffset() {
        this._timeOffset = Date.now();
    }
}
class VisualEngine {
    constructor(context, offset) {
        this._context = context;
        this._offset = offset;
    }
    get context() {
        return this._context;
    }
    get offset() {
        return this._offset;
    }
    drawBody(body) {
        this._context.save();
        this._context.fillStyle = "white";
        this._context.beginPath();
        this._context.arc(body.position.x + this._offset.x, body.position.y + this._offset.y, 25, 0, Math.PI * 2);
        this._context.fill();
        this._context.restore();
    }
    drawVelocity(body) {
        this._context.save();
        const vectorStart = new DOMPoint(body.position.x + this._offset.x, body.position.y + this._offset.y);
        const vectorEnd = new DOMPoint(body.position.x + body.parameters.fullVelocity.x + this._offset.x, body.position.y + body.parameters.fullVelocity.y + this._offset.y);
        const arrowSize = 10;
        const arrowAngle = Math.PI / 4;
        const vectorAngle = Math.atan2(vectorEnd.y - vectorStart.y, vectorEnd.x - vectorStart.x);
        const arrowEnd_1 = new DOMPoint(vectorEnd.x + -Math.cos(vectorAngle - arrowAngle) * arrowSize, vectorEnd.y + -Math.sin(vectorAngle - arrowAngle) * arrowSize);
        const arrowEnd_2 = new DOMPoint(vectorEnd.x + -Math.cos(vectorAngle + arrowAngle) * arrowSize, vectorEnd.y + -Math.sin(vectorAngle + arrowAngle) * arrowSize);
        this._context.strokeStyle = "red";
        this._context.beginPath();
        this._context.moveTo(vectorStart.x, vectorStart.y);
        this._context.lineTo(vectorEnd.x, vectorEnd.y);
        this._context.lineTo(arrowEnd_1.x, arrowEnd_1.y);
        this._context.moveTo(vectorEnd.x, vectorEnd.y);
        this._context.lineTo(arrowEnd_2.x, arrowEnd_2.y);
        this._context.stroke();
        this._context.restore();
    }
    drawPath(body) {
        this._context.save();
        if (body.path.length > 0) {
            this._context.strokeStyle = "gray";
            for (let index = 1; index < body.path.length; index++) {
                this._context.beginPath();
                this._context.globalAlpha = index / body.path.length;
                this._context.moveTo(body.path[index - 1].x + this._offset.x, body.path[index - 1].y + this._offset.y);
                this._context.lineTo(body.path[index].x + this._offset.x, body.path[index].y + this._offset.y);
                this._context.stroke();
            }
        }
        this._context.restore();
    }
}
class Playground {
    constructor(canvas, context, physicalEngine, visualEngine) {
        this._canvas = canvas;
        this._context = context;
        this._physicalEngine = physicalEngine;
        this._visualEngine = visualEngine;
        this._onUpdate = [];
        setInterval(() => {
            this._onUpdate.forEach((handler) => {
                handler(this);
            });
        }, 10);
    }
    get canvas() {
        return this._canvas;
    }
    get context() {
        return this._context;
    }
    get physicalEngine() {
        return this._physicalEngine;
    }
    get visualEngine() {
        return this._visualEngine;
    }
    get editedBody() {
        return this._editedBody;
    }
    set editedBody(value) {
        this._editedBody = value;
    }
    addEventListener(type, handler) {
        switch (type) {
            case "update":
                this._onUpdate.push(handler);
        }
    }
    removeEventListener(type, handler) {
        switch (type) {
            case "update":
                this._onUpdate.push(handler);
        }
    }
}
function draw(playground) {
    playground.physicalEngine.update();
    playground.context.fillStyle = "black";
    playground.context.fillRect(0, 0, window.screen.width, window.screen.height);
    playground.physicalEngine.bodyies.forEach((body) => {
        playground.visualEngine.drawBody(body);
        playground.visualEngine.drawVelocity(body);
        playground.visualEngine.drawPath(body);
    });
    if (playground.editedBody) {
        playground.visualEngine.drawBody(playground.editedBody);
    }
}
function addBodyMass(playground) {
    if (playground.editedBody) {
        playground.editedBody.mass += 0.01e15;
    }
}
function mouseDownHandler(playground) {
    return (function (event) {
        this.editedBody = new PhysicalBody(new DOMPoint(event.offsetX - playground.visualEngine.offset.x, event.offsetY - playground.visualEngine.offset.y), 0, new BodyParameters());
        this.addEventListener("update", addBodyMass);
    }).bind(playground);
}
function mouseMoveHandler(playground) {
    let timeOffset = Date.now();
    return (function (event) {
        if (this.editedBody) {
            const timeDelta = (Date.now() - timeOffset) / 100;
            const newPosition = new DOMPoint(event.offsetX - playground.visualEngine.offset.x, event.offsetY - playground.visualEngine.offset.y);
            this.editedBody.parameters = new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint((newPosition.x - this.editedBody.position.x) / timeDelta, (newPosition.y - this.editedBody.position.y) / timeDelta), new DOMPoint());
            this.editedBody.position = newPosition;
            timeOffset = Date.now();
        }
    }).bind(playground);
}
function mouseUpHandler(playground) {
    return (function (_event) {
        if (this.editedBody) {
            this.physicalEngine.bodyies.push(this.editedBody);
            this.removeEventListener("update", addBodyMass);
            this.editedBody = undefined;
        }
    }).bind(playground);
}
window.onload = () => {
    const physicalEngine = new PhysicalEngine([new PhysicalBody(new DOMPoint(0, 0), 20e15, new BodyParameters()), new PhysicalBody(new DOMPoint(200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, 100), new DOMPoint())), new PhysicalBody(new DOMPoint(-200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, -100), new DOMPoint()))]);
    const canvas = document.getElementById("cnvs");
    const context = canvas?.getContext("2d");
    canvas.width = window.screen.availWidth;
    canvas.height = window.screen.availHeight;
    if (context) {
        const visualEngine = new VisualEngine(context, new DOMPoint(window.screen.availWidth / 2, window.screen.availHeight / 2));
        const playground = new Playground(canvas, context, physicalEngine, visualEngine);
        playground.addEventListener("update", draw);
        canvas.addEventListener("mousedown", mouseDownHandler(playground));
        canvas.addEventListener("mousemove", mouseMoveHandler(playground));
        canvas.addEventListener("mouseup", mouseUpHandler(playground));
    }
};
//# sourceMappingURL=app.js.map