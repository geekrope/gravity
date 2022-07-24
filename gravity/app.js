"use strict";
class BodyParameters {
    constructor(acceleration = new DOMPoint(), velocity = new DOMPoint(), baseVelocity = new DOMPoint(), movement = new DOMPoint()) {
        this.acceleration = acceleration;
        this.velocity = velocity;
        this.baseVelocity = baseVelocity;
        this.movement = movement;
    }
}
class ReadonlyBodyParameters {
    constructor(acceleration = new DOMPoint(), velocity = new DOMPoint(), baseVelocity = new DOMPoint(), movement = new DOMPoint()) {
        this.acceleration = acceleration;
        this.velocity = velocity;
        this.baseVelocity = baseVelocity;
        this.movement = movement;
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
    get bodyes() {
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
    computeVelocity(body, timeDelta, acceleration = undefined) {
        if (this.exists(body)) {
            const newAcceleration = acceleration || this.computeAcceleration(body);
            // finds area (difference between antiderivatives) between current acceleration and last captured acceleration
            return new DOMPoint(Utils.trapezeArea(body.parameters.acceleration.x, newAcceleration.x, timeDelta), Utils.trapezeArea(body.parameters.acceleration.y, newAcceleration.y, timeDelta)); // first integral of acceleration
        }
        else {
            throw new Error("Can't find body");
        }
    }
    computeFullVelocity(baseVelocity, velocity) {
        // adds base velocity to accelerated velocity
        return new DOMPoint(velocity.x + baseVelocity.x, velocity.y + baseVelocity.y);
    }
    computeMovement(body, timeDelta, acceleration = undefined, velocity = undefined) {
        if (this.exists(body)) {
            const newVelocity = velocity || this.computeVelocity(body, timeDelta, acceleration);
            // finds area (difference between antiderivatives) between current velocity and last captured velocity
            return new DOMPoint(Utils.trapezeArea(body.parameters.velocity.x, newVelocity.x, timeDelta), Utils.trapezeArea(body.parameters.velocity.y, newVelocity.y, timeDelta));
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
                const movement = this.computeMovement(body, computedTimeDelta / 1000, acceleration, this.computeFullVelocity(body.parameters.baseVelocity, velocity));
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
    constructor(context) {
        this._context = context;
        this._offset = new DOMPoint(window.screen.availWidth / 2, window.screen.availHeight / 2);
    }
    get context() {
        return this._context;
    }
    drawBody(body) {
        this._context.fillStyle = "white";
        this._context.beginPath();
        this._context.arc(body.position.x + this._offset.x, body.position.y + this._offset.y, 25, 0, Math.PI * 2);
        this._context.fill();
    }
    drawVelocity(body) {
        const vectorStart = new DOMPoint(body.position.x + this._offset.x, body.position.y + this._offset.y);
        const vectorEnd = new DOMPoint(body.position.x + body.parameters.velocity.x + body.parameters.baseVelocity.x + this._offset.x, body.position.y + body.parameters.velocity.y + body.parameters.baseVelocity.y + this._offset.y);
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
    }
    drawPath(body) {
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
    }
}
window.onload = () => {
    const engine = new PhysicalEngine([new PhysicalBody(new DOMPoint(0, 0), 20e15, new BodyParameters()), new PhysicalBody(new DOMPoint(200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, 100), new DOMPoint())), new PhysicalBody(new DOMPoint(-200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, -100), new DOMPoint()))]);
    const canvas = document.getElementById("cnvs");
    const ctx = canvas?.getContext("2d");
    canvas.width = window.screen.availWidth;
    canvas.height = window.screen.availHeight;
    engine.speed = 5;
    if (ctx) {
        const vis = new VisualEngine(ctx);
        setInterval(() => {
            engine.update();
            ctx.fillStyle = "black";
            ctx.fillRect(0, 0, 2560, 1440);
            engine.bodyes.forEach((body) => { vis.drawBody(body); vis.drawVelocity(body); vis.drawPath(body); });
        }, 10);
    }
};
//# sourceMappingURL=app.js.map