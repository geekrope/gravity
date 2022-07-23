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
class PhysicalBody {
    constructor(position, mass, parameters) {
        this._force = new DOMPoint();
        this._parameters = parameters;
        this._path = [];
        this._position = position;
        this._mass = mass;
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
        return this._path;
    }
    set position(position) {
        this._path.push(this._position);
        this._position = position;
    }
    set mass(mass) {
        this._mass = mass;
    }
    set parameters(parameters) {
        this._parameters = parameters;
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
class Utils {
    static distance(point1, point2) {
        return Math.sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
    }
    static trapezeArea(base1, base2, height) {
        return (base1 + base2) * height / 2;
    }
}
class Engine {
    constructor(bodyes) {
        this.G = 6.68e-11;
        this.maxTimeDelta = 10;
        this._bodyes = bodyes;
        this._timeOffset = Date.now();
    }
    get bodyes() {
        return this._bodyes;
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
    computeFullVelocity(body, velocity) {
        if (this.exists(body)) {
            // adds base velocity to accelerated velocity
            return new DOMPoint(velocity.x + body.parameters.baseVelocity.x, velocity.y + body.parameters.baseVelocity.y);
        }
        else {
            throw new Error("Can't find body");
        }
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
        const timeDelta = Date.now() - this._timeOffset;
        this._bodyes.forEach((body) => {
            for (let chunk = 0; chunk < Math.floor(timeDelta / this.maxTimeDelta) + 1; chunk++) {
                const computedTimeDelta = Math.min(this.maxTimeDelta, timeDelta - chunk * this.maxTimeDelta);
                this.computeForce(body);
                const acceleration = this.computeAcceleration(body);
                const velocity = this.computeVelocity(body, computedTimeDelta / 1000, acceleration);
                const movement = this.computeMovement(body, computedTimeDelta / 1000, acceleration, this.computeFullVelocity(body, velocity));
                body.position = new DOMPoint(movement.x + body.position.x, movement.y + body.position.y);
                // area addition rule (S = S1 + S2);
                body.parameters = new BodyParameters(acceleration, new DOMPoint(body.parameters.velocity.x + velocity.x, body.parameters.velocity.y + velocity.y), body.parameters.baseVelocity, new DOMPoint(body.parameters.movement.x + movement.x, body.parameters.movement.y + movement.y));
            }
        });
        this._timeOffset = Date.now();
    }
    start() {
        this._timeOffset = Date.now();
    }
}
class VisualEngine {
    constructor(context) {
        this._context = context;
        this._offset = new DOMPoint(screen.width / 2, screen.height / 2);
    }
    get context() {
        return this._context;
    }
    drawBody(body) {
        this._context.fillStyle = "black";
        this._context.beginPath();
        this._context.arc(body.position.x + this._offset.x, body.position.y + this._offset.y, 25, 0, Math.PI * 2);
        this._context.fill();
    }
    drawVelocity(body) {
        this._context.strokeStyle = "red";
        this._context.beginPath();
        this._context.moveTo(body.position.x + this._offset.x, body.position.y + this._offset.y);
        this._context.lineTo(body.position.x + body.parameters.velocity.x + body.parameters.baseVelocity.x + this._offset.x, body.position.y + body.parameters.velocity.y + body.parameters.baseVelocity.y + this._offset.y);
        this._context.stroke();
    }
    drawMovement(body) {
        this._context.strokeStyle = "green";
        this._context.beginPath();
        this._context.moveTo(body.position.x + this._offset.x, body.position.y + this._offset.y);
        this._context.lineTo(body.position.x + body.parameters.movement.x + this._offset.x, body.position.y + body.parameters.movement.y + this._offset.y);
        this._context.stroke();
    }
    drawPath(body) {
        if (body.path.length > 0) {
            this._context.strokeStyle = "gray";
            this._context.beginPath();
            this._context.moveTo(body.path[0].x + this._offset.x, body.path[0].y + this._offset.y);
            for (let index = 0; index < body.path.length; index++) {
                this._context.lineTo(body.path[index].x + this._offset.x, body.path[index].y + this._offset.y);
            }
            this._context.stroke();
        }
    }
}
class Playground {
    constructor(massUnit, context) {
        this._massUnit = massUnit;
        this._visualEngine = new VisualEngine(context);
        this._physicalEngine = new Engine([]);
        this._newBody = undefined;
        this._interval = -1;
    }
    initNewBody() {
        this._newBody = new PhysicalBody(new DOMPoint(), 0, new BodyParameters());
    }
    addNewBody() {
        if (this._newBody) {
            this._physicalEngine.bodyes.push(this._newBody);
            this._newBody = undefined;
        }
        else {
            throw new Error("New body is not initilized");
        }
    }
    start() {
        this._interval = setInterval(() => {
            this._physicalEngine.update();
            this._visualEngine.context.clearRect(0, 0, 2560, 1440);
            this._physicalEngine.bodyes.forEach((body) => { this._visualEngine.drawBody(body); });
        });
    }
    stop() {
        clearInterval(this._interval);
    }
    set bodyMass(massUnits) {
        if (this._newBody && massUnits) {
            this._newBody.mass = massUnits * this._massUnit;
        }
        else {
            throw new Error("Body or massUnits are not initialized");
        }
    }
    set bodyPosition(position) {
        if (this._newBody && position) {
            this._newBody.position = position;
        }
        else {
            throw new Error("Body or position are not initialized");
        }
    }
    set bodyBaseVelocity(velocity) {
        if (this._newBody && velocity) {
            this._newBody.parameters = new BodyParameters(new DOMPoint(), new DOMPoint(), velocity, new DOMPoint());
        }
        else {
            throw new Error("Body or velocity are not initialized");
        }
    }
    get bodyMass() {
        if (this._newBody) {
            return this._newBody.mass / this._massUnit;
        }
        else {
            return undefined;
        }
    }
    get bodyPosition() {
        if (this._newBody) {
            return this._newBody.position;
        }
        else {
            return undefined;
        }
    }
    get bodyBaseVelocity() {
        if (this._newBody) {
            return this._newBody.parameters.baseVelocity;
        }
        else {
            return undefined;
        }
    }
    get massUnit() {
        return this._massUnit;
    }
}
window.onload = () => {
    const engine = new Engine([new PhysicalBody(new DOMPoint(0, 0), 20e15, new BodyParameters()), new PhysicalBody(new DOMPoint(200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, 100), new DOMPoint())), new PhysicalBody(new DOMPoint(-200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, -100), new DOMPoint()))]);
    const canvas = document.getElementById("cnvs");
    const ctx = canvas?.getContext("2d");
    if (ctx) {
        const vis = new VisualEngine(ctx);
        setInterval(() => {
            engine.update();
            ctx.clearRect(0, 0, 2560, 1440);
            engine.bodyes.forEach((body) => { vis.drawBody(body); vis.drawVelocity(body); vis.drawPath(body); });
        }, 10);
    }
};
//# sourceMappingURL=app.js.map