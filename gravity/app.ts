interface IParameters
{
	acceleration: DOMPoint;
	velocity: DOMPoint;
	baseVelocity: DOMPoint;
	movement: DOMPoint;
}

class BodyParameters implements IParameters
{
	public acceleration: DOMPoint;
	public velocity: DOMPoint;
	public baseVelocity: DOMPoint;
	public movement: DOMPoint;

	public constructor(acceleration: DOMPoint = new DOMPoint(), velocity: DOMPoint = new DOMPoint(), baseVelocity: DOMPoint = new DOMPoint(), movement: DOMPoint = new DOMPoint())
	{
		this.acceleration = acceleration;
		this.velocity = velocity;
		this.baseVelocity = baseVelocity;
		this.movement = movement;
	}
}
class ReadonlyBodyParameters implements IParameters
{
	public readonly acceleration: DOMPoint;
	public readonly velocity: DOMPoint;
	public readonly baseVelocity: DOMPoint;
	public readonly movement: DOMPoint;

	public constructor(acceleration: DOMPoint = new DOMPoint(), velocity: DOMPoint = new DOMPoint(), baseVelocity: DOMPoint = new DOMPoint(), movement: DOMPoint = new DOMPoint())
	{
		this.acceleration = acceleration;
		this.velocity = velocity;
		this.baseVelocity = baseVelocity;
		this.movement = movement;
	}
}

class PhysicalBody
{
	private _force: DOMPoint;
	private _position: DOMPoint;
	private _path: DOMPoint[];
	private _mass: number;
	private _parameters: BodyParameters;

	public get force(): DOMPointReadOnly
	{
		return this._force;
	}
	public get position(): DOMPointReadOnly
	{
		return this._position as DOMPointReadOnly;
	}
	public get mass(): number
	{
		return this._mass;
	}
	public get parameters(): ReadonlyBodyParameters
	{
		return <ReadonlyBodyParameters>this._parameters;
	}
	public get path(): ReadonlyArray<DOMPoint>
	{
		return this._path;
	}

	public set position(position: DOMPointReadOnly)
	{
		this._path.push(this._position);
		this._position = position;
	}
	public set mass(mass: number)
	{
		this._mass = mass;
	}
	public set parameters(parameters: ReadonlyBodyParameters)
	{
		this._parameters = parameters;
	}

	public addForce(force: DOMPointReadOnly): void
	{
		this._force.x += force.x;
		this._force.y += force.y;
	}
	public setForce(force: DOMPointReadOnly): void
	{
		this._force = force;
	}
	public clearForces()
	{
		this._force = new DOMPoint();
	}

	public constructor(position: DOMPoint, mass: number, parameters: BodyParameters)
	{
		this._force = new DOMPoint();
		this._parameters = parameters;
		this._path = [];
		this._position = position;
		this._mass = mass;
	}
}

class Utils
{
	public static distance(point1: DOMPoint, point2: DOMPoint)
	{
		return Math.sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	}
	public static trapezeArea(base1: number, base2: number, height: number)
	{
		return (base1 + base2) * height / 2;
	}
}

class PhysicalEngine
{
	private readonly G: number = 6.68e-11;
	private readonly maxTimeDelta: number = 10;
	private _bodyes: PhysicalBody[];
	private _timeOffset: number;

	public get bodyes(): PhysicalBody[]
	{
		return this._bodyes;
	}

	private exists(body: PhysicalBody): boolean
	{
		const index = this._bodyes.indexOf(body);

		return index != -1;
	}
	private getBodyIndex(body: PhysicalBody): number
	{
		const index = this._bodyes.indexOf(body);

		if (index != -1)
		{
			return index
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeForce(influencedBody: PhysicalBody): void
	{
		const formula = (mass1: number, mass2: number, distance: number) =>
		{
			return this.G * mass1 * mass2 / (distance * distance);
		}

		const influencedBodyIndex = this.getBodyIndex(influencedBody);

		influencedBody.clearForces();

		for (let influencingBodyIndex = 0; influencingBodyIndex < this._bodyes.length; influencingBodyIndex++)
		{
			const influencingBody = this._bodyes[influencingBodyIndex];

			if (influencingBodyIndex != influencedBodyIndex)
			{
				const forceValue = formula(influencedBody.mass, influencingBody.mass, Utils.distance(influencedBody.position, influencingBody.position));
				const angle = Math.atan2(influencingBody.position.y - influencedBody.position.y, influencingBody.position.x - influencedBody.position.x);
				const forceVector = new DOMPoint(forceValue * Math.cos(angle), forceValue * Math.sin(angle));

				influencedBody.addForce(forceVector);
			}
		}
	}
	private computeAcceleration(body: PhysicalBody): DOMPoint
	{
		if (this.exists(body))
		{
			// F = ma
			return new DOMPoint(body.force.x / body.mass, body.force.y / body.mass);
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeVelocity(body: PhysicalBody, timeDelta: number, acceleration: DOMPoint | undefined = undefined): DOMPoint
	{
		if (this.exists(body))
		{
			const newAcceleration = acceleration || this.computeAcceleration(body);

			// finds area (difference between antiderivatives) between current acceleration and last captured acceleration
			return new DOMPoint(Utils.trapezeArea(body.parameters.acceleration.x, newAcceleration.x, timeDelta), Utils.trapezeArea(body.parameters.acceleration.y, newAcceleration.y, timeDelta)); // first integral of acceleration
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeFullVelocity(body: PhysicalBody, velocity: DOMPoint): DOMPoint
	{
		if (this.exists(body))
		{
			// adds base velocity to accelerated velocity
			return new DOMPoint(velocity.x + body.parameters.baseVelocity.x, velocity.y + body.parameters.baseVelocity.y);
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeMovement(body: PhysicalBody, timeDelta: number, acceleration: DOMPoint | undefined = undefined, velocity: DOMPoint | undefined = undefined): DOMPoint
	{
		if (this.exists(body))
		{
			const newVelocity = velocity || this.computeVelocity(body, timeDelta, acceleration);

			// finds area (difference between antiderivatives) between current velocity and last captured velocity
			return new DOMPoint(Utils.trapezeArea(body.parameters.velocity.x, newVelocity.x, timeDelta), Utils.trapezeArea(body.parameters.velocity.y, newVelocity.y, timeDelta));
		}
		else
		{
			throw new Error("Can't find body");
		}
	}

	public update()
	{
		const timeDelta = Date.now() - this._timeOffset;

		this._bodyes.forEach((body) =>
		{
			for (let chunk = 0; chunk < Math.floor(timeDelta / this.maxTimeDelta) + 1; chunk++)
			{
				const computedTimeDelta = Math.min(this.maxTimeDelta, timeDelta - chunk * this.maxTimeDelta);

				this.computeForce(body);
				const acceleration = this.computeAcceleration(body);
				const velocity = this.computeVelocity(body, computedTimeDelta / 1000, acceleration);
				const movement = this.computeMovement(body, computedTimeDelta / 1000, acceleration, this.computeFullVelocity(body, velocity));

				body.position = new DOMPoint(movement.x + body.position.x, movement.y + body.position.y);
				// area addition rule (S = S1 + S2);
				body.parameters = new BodyParameters(
					acceleration,
					new DOMPoint(body.parameters.velocity.x + velocity.x, body.parameters.velocity.y + velocity.y),
					body.parameters.baseVelocity,
					new DOMPoint(body.parameters.movement.x + movement.x, body.parameters.movement.y + movement.y)
				);
			}
		});

		this._timeOffset = Date.now();
	}
	public start()
	{
		this._timeOffset = Date.now();
	}

	public constructor(bodyes: PhysicalBody[])
	{
		this._bodyes = bodyes;
		this._timeOffset = Date.now();
	}
}

class VisualEngine
{
	private _context: CanvasRenderingContext2D;
	private readonly _offset: DOMPoint;

	public get context(): CanvasRenderingContext2D
	{
		return this._context;
	}

	public drawBody(body: PhysicalBody)
	{
		this._context.fillStyle = "black";
		this._context.beginPath();
		this._context.arc(body.position.x + this._offset.x, body.position.y + this._offset.y, 25, 0, Math.PI * 2);
		this._context.fill();
	}
	public drawVelocity(body: PhysicalBody)
	{
		this._context.strokeStyle = "red";
		this._context.beginPath();
		this._context.moveTo(body.position.x + this._offset.x, body.position.y + this._offset.y);
		this._context.lineTo(body.position.x + body.parameters.velocity.x + body.parameters.baseVelocity.x + this._offset.x, body.position.y + body.parameters.velocity.y + body.parameters.baseVelocity.y + this._offset.y);
		this._context.stroke();
	}
	public drawMovement(body: PhysicalBody)
	{
		this._context.strokeStyle = "green";
		this._context.beginPath();
		this._context.moveTo(body.position.x + this._offset.x, body.position.y + this._offset.y);
		this._context.lineTo(body.position.x + body.parameters.movement.x + this._offset.x, body.position.y + body.parameters.movement.y + this._offset.y);
		this._context.stroke();
	}
	public drawPath(body: PhysicalBody)
	{
		if (body.path.length > 0)
		{
			this._context.strokeStyle = "gray";
			this._context.beginPath();
			this._context.moveTo(body.path[0].x + this._offset.x, body.path[0].y + this._offset.y);

			for (let index = 0; index < body.path.length; index++)
			{
				this._context.lineTo(body.path[index].x + this._offset.x, body.path[index].y + this._offset.y);
			}

			this._context.stroke();
		}
	}

	public constructor(context: CanvasRenderingContext2D)
	{
		this._context = context;
		this._offset = new DOMPoint(screen.width / 2, screen.height / 2);
	}
}

class Playground
{
	private _massUnit: number;
	private _visualEngine: VisualEngine;
	private _physicalEngine: PhysicalEngine;
	private _newBody?: PhysicalBody;
	private _interval: number;

	public initNewBody(): void
	{
		this._newBody = new PhysicalBody(new DOMPoint(), 0, new BodyParameters());
	}
	public addNewBody(): void
	{
		if (this._newBody)
		{
			this._physicalEngine.bodyes.push(this._newBody);
			this._newBody = undefined;
		}
		else
		{
			throw new Error("New body is not initilized");
		}
	}

	public start()
	{
		this._interval = setInterval(() =>
		{
			this._physicalEngine.update();

			this._visualEngine.context.clearRect(0, 0, 2560, 1440);

			this._physicalEngine.bodyes.forEach((body) => { this._visualEngine.drawBody(body); });
		})
	}
	public stop()
	{
		clearInterval(this._interval);
	}

	public set bodyMass(massUnits: number | undefined)
	{
		if (this._newBody && massUnits)
		{
			this._newBody.mass = massUnits * this._massUnit;
		}
		else
		{
			throw new Error("Body or massUnits are not initialized");
		}
	}
	public set bodyPosition(position: DOMPointReadOnly | undefined)
	{
		if (this._newBody && position)
		{
			this._newBody.position = position;
		}
		else
		{
			throw new Error("Body or position are not initialized");
		}
	}
	public set bodyBaseVelocity(velocity: DOMPointReadOnly | undefined)
	{
		if (this._newBody && velocity)
		{
			this._newBody.parameters = new BodyParameters(new DOMPoint(), new DOMPoint(), velocity, new DOMPoint());
		}
		else
		{
			throw new Error("Body or velocity are not initialized");
		}
	}

	public get bodyMass(): number | undefined
	{
		if (this._newBody)
		{
			return this._newBody.mass / this._massUnit;
		}
		else
		{
			return undefined;
		}
	}
	public get bodyPosition(): DOMPointReadOnly | undefined
	{
		if (this._newBody)
		{
			return this._newBody.position;
		}
		else
		{
			return undefined;
		}
	}
	public get bodyBaseVelocity(): DOMPointReadOnly | undefined
	{
		if (this._newBody)
		{
			return this._newBody.parameters.baseVelocity;
		}
		else
		{
			return undefined;
		}
	}
	public get massUnit(): number
	{
		return this._massUnit;
	}

	public constructor(massUnit: number, context: CanvasRenderingContext2D)
	{
		this._massUnit = massUnit;
		this._visualEngine = new VisualEngine(context);
		this._physicalEngine = new PhysicalEngine([]);
		this._newBody = undefined;
		this._interval = -1;
	}
}

window.onload = () =>
{
	const engine = new PhysicalEngine([new PhysicalBody(new DOMPoint(0, 0), 20e15, new BodyParameters()), new PhysicalBody(new DOMPoint(200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, 100), new DOMPoint())), new PhysicalBody(new DOMPoint(-200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, -100), new DOMPoint()))]);
	const canvas = <HTMLCanvasElement>document.getElementById("cnvs");
	const ctx = canvas?.getContext("2d");

	if (ctx)
	{
		const vis = new VisualEngine(ctx);

		setInterval(() =>
		{
			engine.update();

			ctx.clearRect(0, 0, 2560, 1440);

			engine.bodyes.forEach((body) => { vis.drawBody(body); vis.drawVelocity(body); vis.drawPath(body) });
		}, 10);
	}
}