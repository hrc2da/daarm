async function parseJson(filename, w, h, padding) {
    const data = await d3.json(filename);
    eoss_results = data[0];
    pretrained_results = data[1];
    model_results = [];
    for (let i = 2; i < data.length; i++) {
        model_results.push(data[i]);
    }

    d3.select("#model_num").attr("max", model_results.length - 1);

    let svg = d3
        .select("#plt_window")
        .append("svg")
        .attrs({
            id: "plt_svg",
            height: h,
            width: w
        });

    let min_cost = 0;
    let min_science = 0;
    let max_cost = 10000;
    let max_science = 0.5;

    let x_scale = d3
        .scaleLinear()
        .domain([min_science, max_science])
        .range([padding, w - padding]);
    let y_scale = d3
        .scaleLinear()
        .domain([min_cost, max_cost])
        .range([h - padding, padding]);
    let x_axis = d3.axisBottom().scale(x_scale);
    let y_axis = d3.axisLeft().scale(y_scale);

    //map functions for scaling pts
    x_map = function(d) {
        // let input = Math.max(Number(d[0]), 0);
        // return x_scale(input);
        return x_scale(d[0]);
    };
    y_map = function(d) {
        // let input = Math.max(Number(d[1]), 0);
        // return y_scale(input);
        return y_scale(d[1]);
    };

    //place x-axis on svg
    svg.append("g")
        .attrs({
            class: "x_axis",
            transform: "translate(0, " + (h - padding) + ")"
        })
        .call(x_axis)
        .append("text")
        .attrs({
            class: "label",
            x: w - padding,
            y: -6
        })
        .style("text-anchor", "end")
        .text("Science Benefit");

    //place y-axis on svg
    svg.append("g")
        .attrs({
            transform: "translate(" + padding + ", 0)"
        })
        .call(y_axis)
        .append("text")
        .attrs({
            class: "label",
            y: padding / 2
        })
        .style("text-anchor", "end")
        .text("Cost");

    //plot eoss plot
    for (let pt of eoss_results.data) {
        svg.append("circle").attrs({
            class: "eoss_pt",
            cx: x_map(pt),
            cy: y_map(pt),
            r: 4,
            fill: "#009999",
            opacity: 0.5
        });
        svg.append("circle").attrs({
            class: "eoss_pt",
            cx: x_map(pt),
            cy: y_map(pt),
            r: 2,
            fill: "#ffffff"
        });
    }
    //plot pretrained plot
    for (let pt of pretrained_results.data) {
        svg.append("rect").attrs({
            class: "pretrained_pt",
            x: x_map(pt),
            y: y_map(pt),
            width: 5,
            height: 5,
            fill: "#ffaa00",
            opacity: 0.5
        });
    }
    //plot first model plot
    let currModel = 0;
    for (let pt of model_results[currModel].data) {
        svg.append("circle").attrs({
            class: "model_pt",
            cx: x_map(pt),
            cy: y_map(pt),
            r: 2,
            fill: "#dc0055"
        });
    }

    //define position of slider based on size of plot
    d3.select("#model_num")
        .style("width", w - 3.75 * padding + "px")
        .style("margin-left", padding + "px");

    // define key
    svg.append("rect").attrs({
        x: w - 210,
        y: 110,
        width: 160,
        height: 75,
        fill: "#ffffff",
        stroke: "#7b7b7b"
    });
    svg.append("circle").attrs({
        cx: w - 200,
        cy: 125,
        r: 4,
        fill: "#009999",
        opacity: 0.7
    });
    svg.append("circle").attrs({
        cx: w - 200,
        cy: 125,
        r: 2,
        fill: "#ffffff"
    });
    svg.append("text")
        .attrs({
            class: "label",
            x: w - 195,
            y: 128
        })
        .text("Evaluated with VASSAR");
    svg.append("rect").attrs({
        x: w - 203,
        y: 145,
        width: 5,
        height: 5,
        fill: "#ffaa00",
        opacity: 0.7
    });
    svg.append("text")
        .attrs({
            class: "label",
            x: w - 195,
            y: 150
        })
        .text("Evaluated with Offline Model");
    svg.append("circle").attrs({
        cx: w - 200,
        cy: 168,
        r: 2,
        fill: "#dc0055"
    });
    svg.append("text")
        .attrs({
            class: "label",
            x: w - 195,
            y: 171
        })
        .text("Evaluated with Current Model");

    // // hide/show buttons
    // svg.append('rect')
    //     .attrs({
    //         id: 'eoss_toggle_btn',
    //         x: w - 210,
    //         y: 200,
    //         width: 130,
    //         height: 25,
    //         fill: "#009999",
    //         stroke: "#7b7b7b"
    //     })
    // svg.append('text')
    //     .attrs({
    //         class: 'button_label',
    //         x: w - 195,
    //         y: 215
    //     })
    //     .text('Hide VASSAR pts')

    // svg.append('rect')
    //     .attrs({
    //         id: 'pretrained_toggle_btn',
    //         x: w - 210,
    //         y: 230,
    //         width: 130,
    //         height: 25,
    //         fill: "#ffaa00",
    //         stroke: "#7b7b7b"
    //     })
    // svg.append('text')
    //     .attrs({
    //         class: 'button_label',
    //         x: w - 195,
    //         y: 245
    //     })
    //     .text('Hide Offline pts')

    // svg.append('rect')
    //     .attrs({
    //         id: 'current_toggle_btn',
    //         x: w - 210,
    //         y: 260,
    //         width: 130,
    //         height: 25,
    //         fill: "#dc0055",
    //         stroke: "#7b7b7b"
    //     })
    // svg.append('text')
    //     .attrs({
    //         class: 'button_label',
    //         x: w - 195,
    //         y: 275
    //     })
    //     .text('Hide Current Model pts')

    // d3.select('#current_toggle_btn').on('mousedown', hidePts('.model_pt'));
    return [eoss_results, pretrained_results, model_results];
}

//update model/model data distribution on slider change.
function updateModel(model_num) {
    svg = d3.select("#plt_svg");
    d3.selectAll(".model_pt").remove();
    for (let pt of model_results[model_num].data) {
        svg.append("circle").attrs({
            class: "model_pt",
            cx: x_map(pt),
            cy: y_map(pt),
            r: 2,
            fill: "#dc0055"
        });
    }
    d3.select("#training_step_label").text("Training Step: " + model_num);
}

function hidePts(class_name) {
    d3.selectAll(class_name).attrs({});
}
