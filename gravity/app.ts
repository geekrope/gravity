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

class ObservableCollection<T>
{
	private _collection: T[];
	private _onChanged?: () => void;

	public get onChanged(): (() => void) | undefined
	{
		return this._onChanged;
	}
	public get collection(): ReadonlyArray<T>
	{
		return this._collection as ReadonlyArray<T>;
	}

	public set onChanged(value: (() => void) | undefined)
	{
		this._onChanged = value;
	}

	private invokeChangedEvent(): void
	{
		if (this._onChanged)
		{
			this._onChanged();
		}
	}

	public add(value: T)
	{
		this._collection.push(value);
		this.invokeChangedEvent();
	}
	public delete(index: number)
	{
		this._collection.splice(index, 1);
		this.invokeChangedEvent();
	}

	constructor()
	{
		this._collection = [];
	}
}

class PhysicalBody
{
	private _force: DOMPoint;
	private _position: DOMPoint;
	private _path: ObservableCollection<DOMPoint>;
	private _mass: number;
	private _parameters: BodyParameters;

	private static readonly maxPathLength = 255;

	public get force(): DOMPointReadOnly
	{
		return this._force as DOMPointReadOnly;
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
		return this._parameters as ReadonlyBodyParameters;
	}
	public get path(): ReadonlyArray<DOMPoint>
	{
		return this._path.collection;
	}

	public set position(position: DOMPointReadOnly)
	{
		this._path.add(this._position);
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

	private reducePath(): void
	{
		if (this._path.collection.length > PhysicalBody.maxPathLength)
		{
			this._path.delete(0);
		}
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
		this._path = new ObservableCollection();
		this._position = position;
		this._mass = mass;

		this._path.onChanged = this.reducePath.bind(this);
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
	private _speed: number;

	public get speed(): number
	{
		return this._speed;
	}
	public get bodyes(): PhysicalBody[]
	{
		return this._bodyes;
	}

	public set speed(value: number)
	{
		this._speed = value;
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
			return index;
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
	private computeFullVelocity(baseVelocity: DOMPoint, velocity: DOMPoint): DOMPoint
	{
		// adds base velocity to accelerated velocity
		return new DOMPoint(velocity.x + baseVelocity.x, velocity.y + baseVelocity.y);
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
		const timeDelta = (Date.now() - this._timeOffset) * this._speed;

		this._bodyes.forEach((body) =>
		{
			for (let chunk = 0; chunk < Math.ceil(timeDelta / this.maxTimeDelta); chunk++)
			{
				const computedTimeDelta = Math.min(this.maxTimeDelta, timeDelta - chunk * this.maxTimeDelta);

				this.computeForce(body);
				const acceleration = this.computeAcceleration(body);
				const velocity = this.computeVelocity(body, computedTimeDelta / 1000, acceleration);
				const movement = this.computeMovement(body, computedTimeDelta / 1000, acceleration, this.computeFullVelocity(body.parameters.baseVelocity, velocity));

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
	public resetTimeOffset()
	{
		this._timeOffset = Date.now();
	}

	public constructor(bodyes: PhysicalBody[])
	{
		this._bodyes = bodyes;
		this._timeOffset = Date.now();
		this._speed = 1;
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

			for (let index = 1; index < body.path.length; index++)
			{
				this._context.beginPath();
				this._context.moveTo(body.path[index - 1].x + this._offset.x, body.path[index - 1].y + this._offset.y);
				this._context.globalAlpha = index / body.path.length;
				this._context.lineTo(body.path[index].x + this._offset.x, body.path[index].y + this._offset.y);
				this._context.stroke();
			}
		}
	}

	public constructor(context: CanvasRenderingContext2D)
	{
		this._context = context;
		this._offset = new DOMPoint(screen.width / 2, screen.height / 2);
	}
}

window.onload = () =>
{
	const engine = new PhysicalEngine([new PhysicalBody(new DOMPoint(0, 0), 20e15, new BodyParameters()), new PhysicalBody(new DOMPoint(200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, 100), new DOMPoint())), new PhysicalBody(new DOMPoint(-200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, -100), new DOMPoint()))]);
	const canvas = <HTMLCanvasElement>document.getElementById("cnvs");
	const ctx = canvas?.getContext("2d");

	engine.speed = 5;

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