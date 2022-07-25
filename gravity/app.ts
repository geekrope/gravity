interface IParameters
{
	acceleration: DOMPoint;
	velocity: DOMPoint;
	baseVelocity: DOMPoint;
	movement: DOMPoint;
	fullVelocity: DOMPoint;
}

class BodyParameters implements IParameters
{
	public acceleration: DOMPoint;
	public velocity: DOMPoint;
	public baseVelocity: DOMPoint;
	public movement: DOMPoint;

	public get fullVelocity(): DOMPoint
	{
		return new DOMPoint(this.baseVelocity.x + this.velocity.x, this.baseVelocity.y + this.velocity.y);
	}

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

	public get fullVelocity(): DOMPoint
	{
		return new DOMPoint(this.baseVelocity.x + this.velocity.x, this.baseVelocity.y + this.velocity.y);
	}

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
	public get bodyies(): PhysicalBody[]
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
	private computeVelocity(body: PhysicalBody, timeDelta: number, acceleration: DOMPoint): DOMPoint
	{
		if (this.exists(body))
		{
			// finds area (difference between antiderivatives) between current acceleration and last captured acceleration
			return new DOMPoint(Utils.trapezeArea(body.parameters.acceleration.x, acceleration.x, timeDelta), Utils.trapezeArea(body.parameters.acceleration.y, acceleration.y, timeDelta)); // first integral of acceleration
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeMovement(body: PhysicalBody, timeDelta: number, velocity: DOMPoint): DOMPoint
	{
		if (this.exists(body))
		{
			// finds area (difference between antiderivatives) between current velocity and last captured velocity
			return new DOMPoint(Utils.trapezeArea(body.parameters.velocity.x, velocity.x, timeDelta), Utils.trapezeArea(body.parameters.velocity.y, velocity.y, timeDelta));
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
				const movement = this.computeMovement(body, computedTimeDelta / 1000, new DOMPoint(velocity.x + body.parameters.baseVelocity.x, velocity.y + body.parameters.baseVelocity.y));

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
	public get offset(): DOMPointReadOnly
	{
		return this._offset as DOMPointReadOnly;
	}

	public drawBody(body: PhysicalBody)
	{
		this._context.save();

		this._context.fillStyle = "white";

		this._context.beginPath();

		this._context.arc(body.position.x + this._offset.x, body.position.y + this._offset.y, 25, 0, Math.PI * 2);

		this._context.fill();

		this._context.restore();
	}
	public drawVelocity(body: PhysicalBody)
	{
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
	public drawPath(body: PhysicalBody)
	{
		this._context.save();

		if (body.path.length > 0)
		{
			this._context.strokeStyle = "gray";

			for (let index = 1; index < body.path.length; index++)
			{
				this._context.beginPath();

				this._context.globalAlpha = index / body.path.length;

				this._context.moveTo(body.path[index - 1].x + this._offset.x, body.path[index - 1].y + this._offset.y);
				this._context.lineTo(body.path[index].x + this._offset.x, body.path[index].y + this._offset.y);

				this._context.stroke();
			}
		}

		this._context.restore();
	}

	public constructor(context: CanvasRenderingContext2D, offset: DOMPoint)
	{
		this._context = context;
		this._offset = offset;
	}
}

type playgroundEvents = "update";

class Playground
{
	private _editedBody?: PhysicalBody;
	private _onUpdate: ((playground: this) => void)[];
	private readonly _canvas: HTMLCanvasElement;
	private readonly _context: CanvasRenderingContext2D;
	private readonly _physicalEngine: PhysicalEngine;
	private readonly _visualEngine: VisualEngine;

	public get canvas(): HTMLCanvasElement
	{
		return this._canvas;
	}
	public get context(): CanvasRenderingContext2D
	{
		return this._context;
	}
	public get physicalEngine(): PhysicalEngine
	{
		return this._physicalEngine;
	}
	public get visualEngine(): VisualEngine
	{
		return this._visualEngine;
	}
	public get editedBody(): PhysicalBody | undefined
	{
		return this._editedBody;
	}

	public set editedBody(value: PhysicalBody | undefined)
	{
		this._editedBody = value;
	}

	public addEventListener(type: "update", handler: (playground: this) => void): void
	public addEventListener(type: playgroundEvents, handler: (playground: this) => void)
	{
		switch (type)
		{
			case "update":
				this._onUpdate.push(handler);
		}
	}

	public removeEventListener(type: "update", handler: (playground: this) => void): void
	public removeEventListener(type: playgroundEvents, handler: (playground: this) => void)
	{
		switch (type)
		{
			case "update":
				this._onUpdate.push(handler);
		}
	}

	public constructor(canvas: HTMLCanvasElement, context: CanvasRenderingContext2D, physicalEngine: PhysicalEngine, visualEngine: VisualEngine)
	{
		this._canvas = canvas;
		this._context = context;
		this._physicalEngine = physicalEngine;
		this._visualEngine = visualEngine;
		this._onUpdate = [];
		setInterval(() =>
		{
			this._onUpdate.forEach((handler) =>
			{
				handler(this);
			});
		}, 10);
	}
}

function draw(playground: Playground)
{
	playground.physicalEngine.update();
	playground.context.fillStyle = "black";
	playground.context.fillRect(0, 0, window.screen.width, window.screen.height);
	
	playground.physicalEngine.bodyies.forEach((body) =>
	{
		playground.visualEngine.drawBody(body);
		playground.visualEngine.drawVelocity(body);
		playground.visualEngine.drawPath(body);
	});

	if (playground.editedBody)
	{
		playground.visualEngine.drawBody(playground.editedBody);
	}	
}
function addBodyMass(playground: Playground)
{
	if (playground.editedBody)
	{
		playground.editedBody.mass += 0.01e15;
	}
}
function mouseDownHandler(playground: Playground): (event: MouseEvent) => void
{
	return (function (this: typeof playground, event: MouseEvent)
	{
		this.editedBody = new PhysicalBody(new DOMPoint(event.offsetX - playground.visualEngine.offset.x, event.offsetY - playground.visualEngine.offset.y), 0, new BodyParameters());
		this.addEventListener("update", addBodyMass);
	}).bind(playground);
}
function mouseMoveHandler(playground: Playground): (event: MouseEvent) => void
{
	let timeOffset = Date.now();
	return (function (this: typeof playground, event: MouseEvent)
	{
		if (this.editedBody)
		{
			const timeDelta = (Date.now() - timeOffset) / 100;
			const newPosition = new DOMPoint(event.offsetX - playground.visualEngine.offset.x, event.offsetY - playground.visualEngine.offset.y);

			this.editedBody.parameters = new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint((newPosition.x - this.editedBody.position.x) / timeDelta, (newPosition.y - this.editedBody.position.y) / timeDelta), new DOMPoint());
			this.editedBody.position = newPosition;

			timeOffset = Date.now();
		}
	}).bind(playground);
}
function mouseUpHandler(playground: Playground): (event: MouseEvent) => void
{
	return (function (this: typeof playground, _event: MouseEvent)
	{
		if (this.editedBody)
		{
			this.physicalEngine.bodyies.push(this.editedBody);
			this.removeEventListener("update", addBodyMass);
			this.editedBody = undefined;
		}
	}).bind(playground);
}

window.onload = () =>
{
	const physicalEngine = new PhysicalEngine([new PhysicalBody(new DOMPoint(0, 0), 20e15, new BodyParameters()), new PhysicalBody(new DOMPoint(200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, 100), new DOMPoint())), new PhysicalBody(new DOMPoint(-200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, -100), new DOMPoint()))]);
	const canvas = <HTMLCanvasElement>document.getElementById("cnvs");
	const context = canvas?.getContext("2d");

	canvas.width = window.screen.availWidth;
	canvas.height = window.screen.availHeight;

	if (context)
	{
		const visualEngine = new VisualEngine(context, new DOMPoint(window.screen.availWidth / 2, window.screen.availHeight / 2));
		const playground = new Playground(canvas, context, physicalEngine, visualEngine);

		playground.addEventListener("update", draw);

		canvas.addEventListener("mousedown", mouseDownHandler(playground));
		canvas.addEventListener("mousemove", mouseMoveHandler(playground));
		canvas.addEventListener("mouseup", mouseUpHandler(playground));
	}
}